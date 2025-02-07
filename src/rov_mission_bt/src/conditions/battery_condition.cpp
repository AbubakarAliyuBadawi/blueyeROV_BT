#include "rov_mission_bt/conditions/battery_condition.hpp"

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

BT::PortsList CheckBatteryLevel::providedPorts() {
    return {};
}

BT::NodeStatus CheckBatteryLevel::tick() {
    if (should_return_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void CheckBatteryLevel::batteryCallback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
    battery_level_ = msg->state_of_charge * 100;
}

void CheckBatteryLevel::returnCallback(const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg) {
    should_return_ = msg->should_return;
}