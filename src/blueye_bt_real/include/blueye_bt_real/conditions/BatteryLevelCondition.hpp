#ifndef BATTERY_LEVEL_CONDITION_HPP
#define BATTERY_LEVEL_CONDITION_HPP

#include <string>
#include <mutex>
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class BatteryLevelCondition : public BT::ConditionNode
{
public:
    BatteryLevelCondition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("threshold", 20.0, "Battery percentage threshold (default: 20%)"),
            BT::OutputPort<double>("battery_level", "Current battery level percentage")
        };
    }
    
    // The tick() method is the core of the behavior
    BT::NodeStatus tick() override;
    
private:
    // ROS subscription
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr battery_sub_;
    
    // Store the latest battery percentage
    double battery_percentage_;
    std::mutex mutex_;
    
    // Callback for battery messages
    void batteryCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    // Flag to know if we've received any battery data
    bool has_battery_data_;
};

#endif // BATTERY_LEVEL_CONDITION_HPP