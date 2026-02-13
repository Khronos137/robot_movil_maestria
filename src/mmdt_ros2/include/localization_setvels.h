#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "localization_utils.h" 

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <algorithm>
#include <iterator>

class LocalizacionSetvelsNode : public rclcpp::Node {
public:
    LocalizacionSetvelsNode();

private:
    // Callbacks
    // --- ¡CORREGIDO! ---
    // (Ahora usa :: en lugar de /)
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg); 
    // --- FIN DE LA CORRECCIÓN ---

    // Publishers y Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

    // Variables de Estado
    GpsConverter gps_converter_;
    geometry_msgs::msg::Quaternion last_orientation_;
    geometry_msgs::msg::Twist real_twist_from_joints_; 

    // Parámetros del Robot
    double wheel_radius_;
    double wheel_separation_;
    
    std::string left_front_joint_name_;
    std::string right_front_joint_name_;
    std::string left_rear_joint_name_;
    std::string right_rear_joint_name_;
};