#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
// CAMBIO 1: Incluir el header de Odometry
#include <nav_msgs/msg/odometry.hpp> 
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

using namespace gazebo;

class GhostTeleport : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);

    std::string topic = "rate_position";
    if (sdf->HasElement("topic")) topic = sdf->Get<std::string>("topic");

    rclcpp::QoS qos(1);
    qos.reliable();
    
    // CAMBIO 2: Cambiar el tipo de dato en la suscripción
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topic, qos, std::bind(&GhostTeleport::PoseCb, this, std::placeholders::_1));

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GhostTeleport::OnUpdate, this));

    model_->SetGravityMode(false);
    RCLCPP_INFO(node_->get_logger(), "GhostTeleport cargado suscrito a: %s", topic.c_str());
  }

private:
  // CAMBIO 3: El callback ahora recibe Odometry
  void PoseCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_odom_ = *msg; 
    have_pose_ = true;
  }

  void OnUpdate() {
    if (!have_pose_) return;
    
    // CAMBIO 4: Nota el acceso doble .pose.pose
    const auto &p = last_odom_.pose.pose.position;
    const auto &q = last_odom_.pose.pose.orientation;
    
    ignition::math::Quaterniond quat(q.w, q.x, q.y, q.z);
    ignition::math::Vector3d pos(p.x, p.y, p.z);
    
    model_->SetWorldPose(ignition::math::Pose3d(pos, quat));
  }

  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr node_;
  // CAMBIO 5: Actualizar tipos de punteros y variables miembro
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry last_odom_; 
  bool have_pose_{false};
  event::ConnectionPtr update_conn_;
};

GZ_REGISTER_MODEL_PLUGIN(GhostTeleport)