// wait_node.cpp
#include "blueye_bt/behaviors/wait_node.hpp"

Wait::Wait(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      duration_seconds_(60) { // Default 60 seconds
}

BT::NodeStatus Wait::onStart() {
    getInput("duration", duration_seconds_);
    
    RCLCPP_INFO(rclcpp::get_logger("wait_node"), 
               "Starting wait for %d seconds", duration_seconds_);
    
    // Record the start time
    start_time_ = rclcpp::Clock().now();
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::onRunning() {
    auto elapsed = (rclcpp::Clock().now() - start_time_).seconds();
    
    if (elapsed >= duration_seconds_) {
        RCLCPP_INFO(rclcpp::get_logger("wait_node"), 
                   "Wait completed after %d seconds", duration_seconds_);
        return BT::NodeStatus::SUCCESS;
    }
    
    // Log progress every 10 seconds
    static int last_report = 0;
    int current_progress = static_cast<int>(elapsed) / 10;
    
    if (current_progress > last_report) {
        last_report = current_progress;
        RCLCPP_INFO(rclcpp::get_logger("wait_node"), 
                   "Still waiting... %.1f / %d seconds", 
                   elapsed, duration_seconds_);
    }
    
    return BT::NodeStatus::RUNNING;
}

void Wait::onHalted() {
    RCLCPP_INFO(rclcpp::get_logger("wait_node"), "Wait halted");
}