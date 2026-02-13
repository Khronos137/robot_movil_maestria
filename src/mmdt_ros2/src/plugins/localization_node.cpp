#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "localization_utils.h"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <iterator>

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization_node"), gps_converter_(0.0, 0.0) {
        
        // --- Declaración de Parámetros (Modelo 4 Ruedas) ---
        // Usamos los valores que me diste como defaults
        this->declare_parameter<double>("wheel_radius", 0.25);
        this->declare_parameter<double>("wheel_separation", 0.7);
        this->declare_parameter<std::string>("left_front_joint_name", "eje1");
        this->declare_parameter<std::string>("right_front_joint_name", "eje2");
        this->declare_parameter<std::string>("left_rear_joint_name", "eje3");
        this->declare_parameter<std::string>("right_rear_joint_name", "eje4");

        // --- Obtención de Parámetros ---
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("wheel_separation", wheel_separation_);
        this->get_parameter("left_front_joint_name", left_front_joint_name_);
        this->get_parameter("right_front_joint_name", right_front_joint_name_);
        this->get_parameter("left_rear_joint_name", left_rear_joint_name_);
        this->get_parameter("right_rear_joint_name", right_rear_joint_name_);

        RCLCPP_INFO(this->get_logger(), "Parámetros de cinemática (4 Ruedas) cargados:");
        RCLCPP_INFO(this->get_logger(), "  Radio Rueda: %.3f m", wheel_radius_);
        RCLCPP_INFO(this->get_logger(), "  Separación Ruedas: %.3f m", wheel_separation_);
        RCLCPP_INFO(this->get_logger(), "  Joints Izquierda (F/R): %s, %s", left_front_joint_name_.c_str(), left_rear_joint_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Joints Derecha (F/R): %s, %s", right_front_joint_name_.c_str(), right_rear_joint_name_.c_str());

        // --- Publishers y Subscribers ---
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&LocalizationNode::gps_callback, this, std::placeholders::_1));

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_plugin/out", 10, std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1));

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&LocalizationNode::joint_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Nodo de localización (4 Ruedas) iniciado.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_orientation_ = msg->orientation;
    }

    // *** MODIFICADO ***
    // Callback para calcular el Twist desde /joint_states (4 Ruedas)
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // 1. Encontrar los índices de las 4 articulaciones
        auto it_lf = std::find(msg->name.begin(), msg->name.end(), left_front_joint_name_);
        auto it_rf = std::find(msg->name.begin(), msg->name.end(), right_front_joint_name_);
        auto it_lr = std::find(msg->name.begin(), msg->name.end(), left_rear_joint_name_);
        auto it_rr = std::find(msg->name.begin(), msg->name.end(), right_rear_joint_name_);

        // 2. Verificar si encontramos TODAS las articulaciones
        if (it_lf == msg->name.end() || it_rf == msg->name.end() || it_lr == msg->name.end() || it_rr == msg->name.end()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "No se encontraron uno o más joints de rueda en /joint_states. Verifique los nombres.");
            return;
        }

        // 3. Obtener los índices
        size_t idx_lf = std::distance(msg->name.begin(), it_lf); // eje1
        size_t idx_rf = std::distance(msg->name.begin(), it_rf); // eje2
        size_t idx_lr = std::distance(msg->name.begin(), it_lr); // eje3
        size_t idx_rr = std::distance(msg->name.begin(), it_rr); // eje4

        // 4. Obtener las velocidades angulares de las ruedas (rad/s)
        double omega_lf = msg->velocity[idx_lf]; // eje1 (izq-frontal)
        double omega_rf = msg->velocity[idx_rf]; // eje2 (der-frontal)
        double omega_lr = msg->velocity[idx_lr]; // eje3 (izq-trasera)
        double omega_rr = msg->velocity[idx_rr]; // eje4 (der-trasera)

        // 5. Calcular la cinemática
        
        // Velocidad angular promedio por lado
        double omega_left_avg = (omega_lf + omega_lr) / 2.0;
        double omega_right_avg = (omega_rf + omega_rr) / 2.0;

        // Velocidad lineal de cada "lado" virtual (v = ω * r)
        double v_left = omega_left_avg * wheel_radius_;
        double v_right = omega_right_avg * wheel_radius_;

        // Velocidad lineal del robot (promedio)
        double v_x = (v_right + v_left) / 2.0;
        // Velocidad angular del robot (diferencia)
        double w_z = (v_right - v_left) / wheel_separation_;


        RCLCPP_WARN(this->get_logger(), 
            "Kinematics Input/Output:\n"
            "  Omegas (LF, RF, LR, RR): [%.3f, %.3f, %.3f, %.3f] rad/s\n"
            "  Calculated (V_x, W_z):   [%.3f m/s, %.3f rad/s]",
            omega_lf, omega_rf, omega_lr, omega_rr, v_x, w_z);

        // 6. Almacenar el resultado
        last_twist_.linear.x = v_x;
        last_twist_.angular.z = w_z;
    }


    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        // (Este callback no cambia en absoluto)
        CartesianPoint pos = gps_converter_.toCartesian(msg->latitude, msg->longitude);
        auto odom_msg = nav_msgs::msg::Odometry();

        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";       
        odom_msg.child_frame_id = "base_link"; 

        odom_msg.pose.pose.position.x = pos.x;
        odom_msg.pose.pose.position.y = pos.y;
        odom_msg.pose.pose.position.z = 0.0; 
        odom_msg.pose.pose.orientation = last_orientation_;
        
        odom_msg.twist.twist = last_twist_; // Usamos el twist calculado

        RCLCPP_WARN(this->get_logger(), 
            "Kinematics Input/Output:\n");

        odom_publisher_->publish(odom_msg);
    }

    // --- Declaraciones de miembros ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    GpsConverter gps_converter_;
    geometry_msgs::msg::Quaternion last_orientation_;
    geometry_msgs::msg::Twist last_twist_;

    // Variables para los parámetros de cinemática
    double wheel_radius_ = 0.0;
    double wheel_separation_ = 0.0;
    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;
};

// --- Función main (sin cambios) ---
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}