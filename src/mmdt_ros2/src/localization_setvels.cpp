#include "localization_setvels.h"

// --- Constructor (MODIFICADO) ---
LocalizacionSetvelsNode::LocalizacionSetvelsNode() : Node("localization_setvels_node"), gps_converter_(0.0, 0.0) {
    // --- Parámetros del Robot ---
    this->declare_parameter<double>("wheel_radius", 0.25); // Usando tus últimos valores
    this->declare_parameter<double>("wheel_separation", 0.7); // Usando tus últimos valores
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    // --- ¡NUEVO! Parámetros de Joints ---
    this->declare_parameter<std::string>("left_front_joint_name", "ejeRueda_FI");
    this->declare_parameter<std::string>("right_front_joint_name", "ejeRueda_FD");
    this->declare_parameter<std::string>("left_rear_joint_name", "ejeRueda_TI");
    this->declare_parameter<std::string>("right_rear_joint_name", "ejeRueda_TD");
    this->get_parameter("left_front_joint_name", left_front_joint_name_);
    this->get_parameter("right_front_joint_name", right_front_joint_name_);
    this->get_parameter("left_rear_joint_name", left_rear_joint_name_);
    this->get_parameter("right_rear_joint_name", right_rear_joint_name_);

    // --- Publicadores ---
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/global", 10);
    wheel_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/controlador_velocidad_ruedas/commands", 10);

    // --- Suscriptores ---
    // (QoS para sensores de Gazebo)
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", sensor_qos, 
        std::bind(&LocalizacionSetvelsNode::gps_callback, this, std::placeholders::_1));
    
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_plugin/out", sensor_qos, 
        std::bind(&LocalizacionSetvelsNode::imu_callback, this, std::placeholders::_1));
    
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_movil", 10, 
        std::bind(&LocalizacionSetvelsNode::twist_callback, this, std::placeholders::_1));

    // *** ¡NUEVA SUBSCRIPCIÓN! ***
    // Aquí es donde leemos los joints reales
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos, // Usamos el QoS de sensor
        std::bind(&LocalizacionSetvelsNode::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo 'localization_setvels' (versión fusionada) iniciado.");
}

// --- imu_callback (Sin cambios) ---
void LocalizacionSetvelsNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_orientation_ = msg->orientation;
}

// --- gps_callback (MODIFICADO) ---
// Ahora usará el twist calculado desde los joints
void LocalizacionSetvelsNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    CartesianPoint pos = gps_converter_.toCartesian(msg->latitude, msg->longitude);
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 1. Llenar la POSE (GPS + IMU)
    odom_msg.pose.pose.position.x = pos.x;
    odom_msg.pose.pose.position.y = pos.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = last_orientation_;

    // 2. *** ¡ARREGLO PRINCIPAL! ***
    //    Usamos el twist REAL calculado desde /joint_states
    odom_msg.twist.twist = real_twist_from_joints_;

    odom_publisher_->publish(odom_msg);
}

// --- twist_callback (Sin cambios) ---
// Este callback *solo* calcula y envía comandos a las ruedas
void LocalizacionSetvelsNode::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    double linear_x = msg->twist.linear.x;
    double angular_z = msg->twist.angular.z;
    double v_right = linear_x + (angular_z * wheel_separation_ / 2.0);
    double v_left = linear_x - (angular_z * wheel_separation_ / 2.0);
    double omega_right = v_right / wheel_radius_;
    double omega_left = v_left / wheel_radius_;
    auto wheel_speeds_msg = std_msgs::msg::Float64MultiArray();
    wheel_speeds_msg.data = {omega_left, omega_right, omega_left, omega_right};
    wheel_cmd_publisher_->publish(wheel_speeds_msg);
}


// *** ¡NUEVO CALLBACK! ***
// Esta es la lógica que querías, tomada de nuestro primer nodo.
void LocalizacionSetvelsNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    // 1. Encontrar los índices
    auto it_lf = std::find(msg->name.begin(), msg->name.end(), left_front_joint_name_);
    auto it_rf = std::find(msg->name.begin(), msg->name.end(), right_front_joint_name_);
    auto it_lr = std::find(msg->name.begin(), msg->name.end(), left_rear_joint_name_);
    auto it_rr = std::find(msg->name.begin(), msg->name.end(), right_rear_joint_name_);

    // 2. Verificar
    if (it_lf == msg->name.end() || it_rf == msg->name.end() || it_lr == msg->name.end() || it_rr == msg->name.end()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "No se encontraron los joints de rueda en /joint_states. Verifique los nombres en el YAML.");
        return;
    }

    // 3. Obtener índices
    size_t idx_lf = std::distance(msg->name.begin(), it_lf); // ejeRueda_FI
    size_t idx_rf = std::distance(msg->name.begin(), it_rf); // ejeRueda_FD
    size_t idx_lr = std::distance(msg->name.begin(), it_lr); // ejeRueda_TI
    size_t idx_rr = std::distance(msg->name.begin(), it_rr); // ejeRueda_TD

    // 4. Obtener velocidades angulares (reales)
    double omega_lf = msg->velocity[idx_lf]; 
    double omega_rf = msg->velocity[idx_rf]; 
    double omega_lr = msg->velocity[idx_lr]; 
    double omega_rr = msg->velocity[idx_rr]; 

    // 5. Calcular cinemática directa (para obtener twist real)
    double omega_left_avg = (omega_lf + omega_lr) / 2.0;
    double omega_right_avg = (omega_rf + omega_rr) / 2.0;
    double v_left = omega_left_avg * wheel_radius_;
    double v_right = omega_right_avg * wheel_radius_;

    double v_x = (v_right + v_left) / 2.0;
    double w_z = (v_right - v_left) / wheel_separation_;

    // Imprimir para depurar
    // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
    //     "[JointState CB] Omegas (LF,RF,LR,RR): [%.2f, %.2f, %.2f, %.2f] | Twist (Vx, Wz): [%.2f, %.2f]",
    //     omega_lf, omega_rf, omega_lr, omega_rr, v_x, w_z);

    // 6. Almacenar el twist REAL
    real_twist_from_joints_.linear.x = v_x;
    real_twist_from_joints_.angular.z = w_z;
}

// --- Función main (Sin cambios) ---
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizacionSetvelsNode>());
    rclcpp::shutdown();
    return 0;
}