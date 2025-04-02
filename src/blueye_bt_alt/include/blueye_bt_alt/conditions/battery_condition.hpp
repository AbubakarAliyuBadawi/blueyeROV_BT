#ifndef BATTERY_CONDITION_HPP
#define BATTERY_CONDITION_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mundus_mir_msgs_alt/msg/battery_status_alt.hpp>
#include <mundus_mir_msgs_alt/msg/return_recommendation_alt.hpp>

class CheckBatteryLevel : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<mundus_mir_msgs_alt::msg::BatteryStatusAlt>::SharedPtr battery_sub_;
    rclcpp::Subscription<mundus_mir_msgs_alt::msg::ReturnRecommendationAlt>::SharedPtr return_sub_;
    bool should_return_;
    double battery_level_;

public:
    CheckBatteryLevel(const std::string& name, const BT::NodeConfiguration& config);
    
    // Updated ports definition for v4
    static BT::PortsList providedPorts() {
        return BT::PortsList(); // More explicit in v4
    }
    
    BT::NodeStatus tick() override;

private:
    void batteryCallback(const mundus_mir_msgs_alt::msg::BatteryStatusAlt::SharedPtr msg);
    void returnCallback(const mundus_mir_msgs_alt::msg::ReturnRecommendationAlt::SharedPtr msg);
};

#endif // BATTERY_CONDITION_HPP