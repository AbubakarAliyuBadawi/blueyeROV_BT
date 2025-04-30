// PipelineInspectionMission.cpp
#include "blueye_bt_real/behaviors/pipeline_inspection_mission.hpp"
#include <cstdlib>
#include <sstream>
#include <rclcpp/rclcpp.hpp>

extern rclcpp::Node::SharedPtr g_node;

PipelineInspectionMission::PipelineInspectionMission(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::NodeStatus PipelineInspectionMission::tick() {
    // Get parameters from ports
    std::string drone_ip = "192.168.1.101";
    getInput<std::string>("drone_ip", drone_ip);
    
    bool goto_docking = true;
    getInput<bool>("goto_docking", goto_docking);
    
    // Build command with parameters
    std::stringstream cmd_ss;
    cmd_ss << "/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/integrated_pipeline_inspection.py "
           << "--drone-ip " << drone_ip << " "
           << "--goto-docking " << (goto_docking ? "true" : "false");
           
    std::string cmd = cmd_ss.str();
    RCLCPP_INFO(g_node->get_logger(), "Executing command: %s", cmd.c_str());
    
    // Execute the pipeline inspection script
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(g_node->get_logger(),
                    "Pipeline inspection mission failed with error code: %d", result);
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(g_node->get_logger(),
               "Pipeline inspection mission completed successfully");
    return BT::NodeStatus::SUCCESS;
}