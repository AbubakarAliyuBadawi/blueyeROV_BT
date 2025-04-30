#include "blueye_bt_real/conditions/BatteryLevelCondition.hpp"
#include <rclcpp/rclcpp.hpp>

extern rclcpp::Node::SharedPtr g_node; // Reference to the global node

BatteryLevelCondition::BatteryLevelCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), 
      battery_percentage_(0.0),
      has_battery_data_(false)
{
    // Subscribe to the battery topic published by blueye_commands.py
    battery_sub_ = g_node->create_subscription<geometry_msgs::msg::Pose>(
        "/blueye/battery", 10, 
        std::bind(&BatteryLevelCondition::batteryCallback, this, std::placeholders::_1));
        
    RCLCPP_INFO(g_node->get_logger(), "BatteryLevelCondition node initialized, waiting for battery data");
}

void BatteryLevelCondition::batteryCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Calculate battery percentage based on runtime_to_empty
    float time_remaining = msg->orientation.x; // runtime_to_empty in seconds
    float full_runtime = 20400.0; // Estimate of 5.6 hours (20400 seconds) at full charge
    
    // Calculate percentage and clamp to reasonable range (0-100%)
    battery_percentage_ = (time_remaining / full_runtime) * 100.0;
    battery_percentage_ = std::max(0.0, std::min(100.0, battery_percentage_));
    
    // Store current, voltage, and other values for debugging if needed
    current_ = msg->position.z;  // Store current for potential future use
    charging_current_ = msg->position.x;  // Store charging current
    
    has_battery_data_ = true;
    RCLCPP_INFO(g_node->get_logger(), "Estimated battery level: %.1f%% (runtime: %.1f s, current: %.2f A)", 
               battery_percentage_, time_remaining, current_);
}

BT::NodeStatus BatteryLevelCondition::tick()
{
    // Get the threshold from input port (default is 20%)
    double threshold = 20.0;
    getInput("threshold", threshold);
    
    // Get current battery level (thread-safe)
    double current_battery_level;
    bool has_data;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_battery_level = battery_percentage_;
        has_data = has_battery_data_;
    }
    
    // First, check if we have received any battery data
    if (!has_data)
    {
        RCLCPP_WARN(g_node->get_logger(), "No battery data received yet");
        return BT::NodeStatus::FAILURE;
    }
    
    // Write the current battery level to the output port
    setOutput("battery_level", current_battery_level);
    
    // Compare battery percentage to threshold
    if (current_battery_level <= threshold)
    {
        RCLCPP_INFO(g_node->get_logger(), "Battery level (%.1f%%) is below threshold (%.1f%%)", 
                    current_battery_level, threshold);
        return BT::NodeStatus::SUCCESS;  // Return SUCCESS when battery is low
    }
    else
    {
        RCLCPP_DEBUG(g_node->get_logger(), "Battery level (%.1f%%) is above threshold (%.1f%%)", 
                     current_battery_level, threshold);
        return BT::NodeStatus::FAILURE;  // Return FAILURE when battery is good
    }
}