// launch_docking_procedure.cpp
#include "blueye_bt/behaviors/launch_docking_procedure.hpp"
#include <cstdlib>

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus LaunchDockingProcedure::tick() {
    RCLCPP_INFO(rclcpp::get_logger("launch_docking"), "Launching integrated docking procedure");
    
    // Launch the docking script as a background process
    std::string cmd = "~/Desktop/blueyeROV_BT/launch_docking.sh &";
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