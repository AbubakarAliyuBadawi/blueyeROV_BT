#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <marine_acoustic_msgs/msg/dvl.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <deque>

class AltitudeController : public rclcpp::Node {
public:
    AltitudeController();

private:
    // Subscribers
    rclcpp::Subscription<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr modified_cmd_vel_pub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    
    // Parameters
    double target_altitude_;
    bool altitude_control_enabled_;
    double heave_speed_;  // Maximum vertical speed
    double min_altitude_; // Safety minimum altitude
    double altitude_safety_margin_;
    
    // State variables
    bool dvl_valid_;
    double current_altitude_;
    double filtered_altitude_;
    bool altitude_filter_initialized_;
    std::deque<float> altitude_buffer_;
    static constexpr size_t ALTITUDE_BUFFER_SIZE = 5;
    rclcpp::Time last_dvl_time_;
    
    // Callback functions
    void dvlCallback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void enableAltitudeControlCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    // Helper functions
    double calculateAltitudeControl(const rclcpp::Time& current_time);
};