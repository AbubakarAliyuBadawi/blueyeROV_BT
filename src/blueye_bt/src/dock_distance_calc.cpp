#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <array>
#include <cmath>
#include <memory>

class DockDistanceCalculator : public rclcpp::Node {
public:
    DockDistanceCalculator() : Node("dock_distance_calculator") {
        // Hardcoded dock station coordinates
        dock_position_ = {-221.0, 59.0, 194.90};

        RCLCPP_INFO(this->get_logger(), "=== UPDATED DOCK COORDINATES (VERSION 2025-03-03) ===");
        RCLCPP_INFO(this->get_logger(), "Dock position set to: [%.2f, %.2f, %.2f]", 
            dock_position_[0], dock_position_[1], dock_position_[2]);
        
        // Create subscription to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/blueye/odometry_frd/gt",
            10,
            std::bind(&DockDistanceCalculator::odometryCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for distance
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/blueye/distance_to_dock",
            10
        );
    }

private:
    std::array<double, 3> dock_position_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract current position
        std::array<double, 3> current_pos = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        };
        
        // Calculate Euclidean distance
        double distance = std::sqrt(
            std::pow(current_pos[0] - dock_position_[0], 2) +
            std::pow(current_pos[1] - dock_position_[1], 2) +
            std::pow(current_pos[2] - dock_position_[2], 2)
        );
        
        // Create and publish distance message
        auto distance_msg = std::make_unique<std_msgs::msg::Float64>();
        distance_msg->data = distance;
        distance_pub_->publish(std::move(distance_msg));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockDistanceCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}