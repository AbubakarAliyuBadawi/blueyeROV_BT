#include "blueye_bt_real/behaviors/launch_depth_mission.hpp"
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

LaunchSimpleDepthMission::LaunchSimpleDepthMission(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchSimpleDepthMission::tick() {
    // Fixed script path - no blackboard parameters
    std::string script_path = "/home/badawi/Desktop/thesis/BlueyeROV_Autonomous_Docking/src/ocean_test/simple_depth_mission.py";
    
    RCLCPP_INFO(rclcpp::get_logger("launch_simple_depth_mission"),
               "Launching simple depth mission with default parameters");
    
    // Build command with default parameters
    std::string cmd = script_path;
    
    RCLCPP_INFO(rclcpp::get_logger("launch_simple_depth_mission"), "Executing command: %s", cmd.c_str());
    
    // Execute depth mission script and wait for completion
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_simple_depth_mission"),
                    "Simple depth mission failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("launch_simple_depth_mission"),
               "Simple depth mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}