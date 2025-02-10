#ifndef ROV_MISSION_BT_BATTERY_CONDITION_HPP
#define ROV_MISSION_BT_BATTERY_CONDITION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs/msg/battery_status.hpp"
#include "mundus_mir_msgs/msg/return_recommendation.hpp"

class CheckBatteryLevel : public BT::ConditionNode {
public:
    CheckBatteryLevel(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

private:
    BT::NodeStatus tick() override;
    void batteryCallback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg);
    void returnCallback(const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg);  // Fixed function name

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<mundus_mir_msgs::msg::ReturnRecommendation>::SharedPtr return_sub_;
    bool should_return_;
    double battery_level_;
};

#endif // ROV_MISSION_BT_BATTERY_CONDITION_HPP