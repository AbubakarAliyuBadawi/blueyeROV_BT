// launch_docking_procedure.cpp
#include "blueye_bt_real/behaviors/launch_docking_real.hpp"
#include <cstdlib>

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchDockingProcedure::tick() {
    RCLCPP_INFO(rclcpp::get_logger("launch_docking"), "Launching docking procedure");
    
    // Launch the docking script as a background process
    std::string cmd = "~/Desktop/blueyeROV_BT/src/bash_scripts/launch_docking_real.sh &";
    int result = system(cmd.c_str());
    
    if (result != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("launch_docking"), 
                    "Failed to launch docking procedure, error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("launch_docking"), 
               "Docking procedure launched successfully");
    
    // Return success immediately - the docking process will run on its own
    return BT::NodeStatus::SUCCESS;
}