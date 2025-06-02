#include "blueye_bt_real/behaviors/launch_mission_procedure.hpp"
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

LaunchMissionProcedure::LaunchMissionProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchMissionProcedure::tick() {
    // Get parameters from ports
    std::string script_path;
    if (!getInput<std::string>("script_path", script_path)) {
        script_path = "~/Desktop/blueyeROV_BT/src/bash_scripts/launch_mission.sh";
    }
    
    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip)) {
        drone_ip = "192.168.1.101";
    }
    
    int max_retries = 5;
    getInput<int>("max_retries", max_retries);

    RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                "Launching pipeline inspection mission with drone IP: %s, max retries: %d",
                drone_ip.c_str(), max_retries);

    // Build command with only the parameters that exist in the new script
    std::stringstream cmd_ss;
    cmd_ss << script_path << " "
           << "--drone-ip " << drone_ip << " "
           << "--max-retries " << max_retries;

    std::string cmd = cmd_ss.str();
    RCLCPP_INFO(rclcpp::get_logger("launch_mission"), "Executing command: %s", cmd.c_str());

    // Execute mission script and wait for completion
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_mission"),
                     "Mission failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("launch_mission"),
                "Mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}