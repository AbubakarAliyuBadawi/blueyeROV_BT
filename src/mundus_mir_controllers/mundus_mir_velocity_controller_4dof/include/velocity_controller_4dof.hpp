// Implementation of a ROS2 node that runs four instances of "pid_siso_cpp"
// controller, with current and desired subscribers based on
// "geometry_msgs/odometry.msg" and publishes wrench messages based on
// "geometry_msgs/msg/WrenchStamped".

// Dependencies:
//  - eigen3
//  - pid_siso_cpp

// Implementation by MF.


#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pid_siso.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

class VelocityController4dof : public rclcpp::Node {
    public:
    VelocityController4dof ();

    private:
    // Odometry msg callback
    void subscriber_odometry_callback_ (const nav_msgs::msg::Odometry::SharedPtr msg);

    // Desired state msg callback
    void subscriber_desired_callback_ (const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // PID computation callback
    void pid_callback_ ();

    // PID SISO controllers
    PID_siso surgeController_, swayController_, depthController_, yawController_;

    // ROS2 Parameters
    int param_qos_buffer_size_, param_controller_frequency_;
    std::string param_topic_sub_odometry_, param_topic_sub_desired_, param_topic_pub_;

    // Parameter - Gain Values
    // Size: 6 elements
    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    Eigen::VectorXd surge_param_, sway_param_, depth_param_, yaw_param_;

    // Current and desired values
    Eigen::Vector4d current_values_, desired_values_;

    // ROS2 Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS2 Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odometry_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_desired_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;

    // ROS2 Parameter callback
    rcl_interfaces::msg::SetParametersResult
    paramsCallback (const std::vector<rclcpp::Parameter>& params);
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
