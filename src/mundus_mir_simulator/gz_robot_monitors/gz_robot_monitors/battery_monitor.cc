#include "gz_robot_monitors/battery_monitor.h"

namespace gz_robot_monitors
{
    BatteryMonitor::BatteryMonitor() : Node("battery_monitor")
    {
        // Initialize subscription and publisher
        subscription_ = this->create_subscription<mundus_mir_msgs::msg::BatteryStatus>(
            "/blueye/battery",
            10,
            [this](const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
                this->battery_callback(msg);
            });

        publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/blueye/monitor/state_of_charge", 10);

        RCLCPP_INFO(this->get_logger(), "Battery Monitor Node has started.");
    }


    void BatteryMonitor::battery_callback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg)
    {
        // Determine the state of charge
        if (msg->state_of_charge > 0.0) {
            current_battery_level_ = msg->state_of_charge;
        } else if (msg->calculated_state_of_charge > 0.0) {
            current_battery_level_ = msg->calculated_state_of_charge;
        } else if (msg->full_charge_capacity > 0.0) {
            current_battery_level_ = msg->remaining_capacity / msg->full_charge_capacity;
        } else {
            RCLCPP_WARN(this->get_logger(), "Full charge capacity is zero; cannot calculate battery level.");
            current_battery_level_ = 0.0;
        }

        // Log the battery level
        if (current_battery_level_ < low_battery_threshold_) {
            RCLCPP_WARN(this->get_logger(), "Battery level low: %.1f%%", current_battery_level_ * 100.0);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Battery level: %.1f%%", current_battery_level_ * 100.0);
        }

        // Publish the state of charge
        std_msgs::msg::Float32 soc_msg;
        soc_msg.data = current_battery_level_;
        publisher_->publish(soc_msg);
    }
}  // namespace gz_robot_monitors
