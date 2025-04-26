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
        script_path = "~/Desktop/blueyeROV_BT/launch_mission.sh";
    }
    
    std::string drone_ip;
    if (!getInput<std::string>("drone_ip", drone_ip)) {
        drone_ip = "192.168.1.101";
    }
    
    double start_lat = 63.441475;
    getInput<double>("start_lat", start_lat);
    
    double start_lon = 10.348348;
    getInput<double>("start_lon", start_lon);
    
    double start_heading = 0.0;
    getInput<double>("start_heading", start_heading);
    
    RCLCPP_INFO(rclcpp::get_logger("launch_mission"), 
                "Launching pipeline inspection mission at coordinates: %f, %f, heading: %f", 
                start_lat, start_lon, start_heading);
    
    // Build command with parameters
    std::stringstream cmd_ss;
    cmd_ss << script_path << " "
           << "--drone-ip " << drone_ip << " "
           << "--start-lat " << start_lat << " "
           << "--start-lon " << start_lon << " "
           << "--start-heading " << start_heading;
    
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