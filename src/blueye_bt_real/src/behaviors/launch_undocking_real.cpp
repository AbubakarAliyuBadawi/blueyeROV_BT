#include "blueye_bt_real/behaviors/launch_undocking_real.hpp"
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

LaunchUndockingProcedure::LaunchUndockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchUndockingProcedure::tick() {
    // Get parameters from ports
    std::string script_path;
    if (!getInput<std::string>("script_path", script_path)) {
        script_path = "/home/badawi/Desktop/blueyeROV_BT/src/bash_scripts/launch_undocking_real.sh";
    }
    
    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip)) {
        drone_ip = "192.168.1.101";
    }
    
    int reverse_duration = 10;
    getInput<int>("reverse_duration", reverse_duration);
    
    float reverse_power = 0.4f;
    getInput<float>("reverse_power", reverse_power);

    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"),
                "Launching undocking procedure with drone IP: %s, duration: %d, power: %.2f",
                drone_ip.c_str(), reverse_duration, reverse_power);

    // Build command with parameters
    std::stringstream cmd_ss;
    cmd_ss << script_path << " "
           << "--drone-ip " << drone_ip << " "
           << "--reverse-duration " << reverse_duration << " "
           << "--reverse-power " << reverse_power;

    std::string cmd = cmd_ss.str();
    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"), "Executing command: %s", cmd.c_str());

    // Execute undocking script and wait for completion
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_undocking"),
                     "Undocking failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("launch_undocking"),
                "Undocking completed successfully");
    return BT::NodeStatus::SUCCESS;
}