#include "gz_robot_monitors/battery_monitor.h"
#include "gz_robot_monitors/geofence_monitor.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create shared pointers for the monitors
    auto battery_monitor = std::make_shared<gz_robot_monitors::BatteryMonitor>();
    auto geofence_monitor = std::make_shared<gz_robot_monitors::GeofenceMonitor>();

    // Create a MultiThreadedExecutor to handle both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(battery_monitor);
    executor.add_node(geofence_monitor);

    rclcpp::Rate rate(0.1); // 0.1 Hz = 1 iteration every 10 seconds

    // Spin the executor
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting both monitors...");
    
    while (rclcpp::ok())
    {
        // Process callbacks for the nodes
        executor.spin_some();

        // Log a message indicating the iteration
        RCLCPP_INFO(rclcpp::get_logger("main"), "Running monitors at 10-second interval.");

        // Sleep for 10 seconds
        rate.sleep();
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

