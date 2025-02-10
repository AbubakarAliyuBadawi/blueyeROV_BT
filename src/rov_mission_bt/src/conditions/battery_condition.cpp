#include "rov_mission_bt/conditions/battery_condition.hpp"

// Constructor remains largely the same
CheckBatteryLevel::CheckBatteryLevel(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), 
      should_return_(false), 
      battery_level_(100.0) {
    
    node_ = rclcpp::Node::make_shared("battery_check_node");
    
    battery_sub_ = node_->create_subscription<mundus_mir_msgs::msg::BatteryStatus>(
        "/blueye/battery", 10,
        std::bind(&CheckBatteryLevel::batteryCallback, this, std::placeholders::_1));
        
    return_sub_ = node_->create_subscription<mundus_mir_msgs::msg::ReturnRecommendation>(
        "/blueye/return_recommendation", 10,
        std::bind(&CheckBatteryLevel::returnCallback, this, std::placeholders::_1));
}

// Remove this function since it's now defined in the header
// BT::PortsList CheckBatteryLevel::providedPorts() {
//     return BT::PortsList({});
// }

BT::NodeStatus CheckBatteryLevel::tick() {
    rclcpp::spin_some(node_);  // Make sure to process callbacks
    
    if (should_return_) {
        RCLCPP_INFO(node_->get_logger(), "Battery check triggered return: Level %.1f%%", battery_level_);
        return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "Battery level: %.1f%% - No return needed", battery_level_);
    return BT::NodeStatus::FAILURE;
}

void CheckBatteryLevel::batteryCallback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
    battery_level_ = msg->state_of_charge * 100;
}

void CheckBatteryLevel::returnCallback(const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg) {
    should_return_ = msg->should_return;
}