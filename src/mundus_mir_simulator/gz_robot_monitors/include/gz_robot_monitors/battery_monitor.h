#ifndef GZ_ROBOT_MONITORS__BATTERY_MONITOR_H_
#define GZ_ROBOT_MONITORS__BATTERY_MONITOR_H_

#include "rclcpp/rclcpp.hpp"
#include "mundus_mir_msgs/msg/battery_status.hpp" 
#include "std_msgs/msg/float32.hpp"               

namespace gz_robot_monitors
{
    class BatteryMonitor : public rclcpp::Node
    {
    public:
        BatteryMonitor();

    private:
        // Callback to process battery status messages
        void battery_callback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg);

        // Subscriber to /blueye/battery
        rclcpp::Subscription<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr subscription_;

        // Publisher for /blueye/monitor/state_of_charge
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

        // Threshold for low battery alert
        const float low_battery_threshold_ = 0.2;

        // Current battery level
        float current_battery_level_ = 1.0;
    };
}  // namespace gz_robot_monitors

#endif  // GZ_ROBOT_MONITORS__BATTERY_MONITOR_H_
