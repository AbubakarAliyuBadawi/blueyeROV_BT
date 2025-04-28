#include "blueye_bt_real/behaviors/launch_depth_mission.hpp"
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

LaunchSimpleDepthMission::LaunchSimpleDepthMission(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchSimpleDepthMission::tick() {
    
    // Get parameters from ports
    std::string script_path;
    if (!getInput<std::string>("script_path", script_path)) {
        script_path = "~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/mission_planner_scripts/goto_depth.py";
    }
    
    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip)) {
        drone_ip = "192.168.1.101";
    }
    
    double start_lat = 63.4406991;
    getInput<double>("start_lat", start_lat);
    
    double start_lon = 10.3489964;
    getInput<double>("start_lon", start_lon);
    
    double start_heading = 0.0;
    getInput<double>("start_heading", start_heading);
    
    double target_depth = 1.3;
    getInput<double>("target_depth", target_depth);
    
    int duration = 120;
    getInput<int>("duration", duration);
    
    RCLCPP_INFO(rclcpp::get_logger("launch_depth_mission"),
               "Launching depth mission at coordinates: %f, %f, heading: %f, target depth: %f, duration: %d seconds",
               start_lat, start_lon, start_heading, target_depth, duration);
    
    // Build command with parameters
    std::stringstream cmd_ss;
    cmd_ss << script_path << " "
           << "--drone-ip " << drone_ip << " "
           << "--start-lat " << start_lat << " "
           << "--start-lon " << start_lon << " "
           << "--start-heading " << start_heading << " "
           << "--depth " << target_depth << " "
           << "--duration " << duration << " ";
    
    std::string cmd = cmd_ss.str();
    RCLCPP_INFO(rclcpp::get_logger("launch_depth_mission"), "Executing command: %s", cmd.c_str());
    
    // Execute depth mission script and wait for completion
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_depth_mission"),
                    "Depth mission failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("launch_depth_mission"),
               "Depth mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}
