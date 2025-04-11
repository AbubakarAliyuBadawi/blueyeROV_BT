#include "blueye_bt_real/actions/set_depth_action.hpp"

namespace blueye_bt_real {

SetDepthAction::SetDepthAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      logger_(rclcpp::get_logger("set_depth_action"))
{
    // Initialize SDK drone reference here if needed
}

BT::PortsList SetDepthAction::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("depth", "Target depth in meters"));
    ports.insert(BT::InputPort<double>("speed", "Vertical speed in m/s"));
    return ports;
}

BT::NodeStatus SetDepthAction::tick()
{
    // Get parameters from ports
    double depth, speed = 0.2;
    
    if (!getInput("depth", depth)) {
        RCLCPP_ERROR(logger_, "Missing required 'depth' input");
        return BT::NodeStatus::FAILURE;
    }
    
    getInput("speed", speed);  // Optional parameter
    
    RCLCPP_INFO(logger_, "Setting depth to %.2f meters with speed %.2f m/s", depth, speed);
    
    try {
        // Here you would set the depth using Blueye SDK
        /*
        // Example SDK code (replace with actual SDK calls)
        auto depth_set_point = blueye::protocol::DepthSetPoint(
            depth,
            speed,
            blueye::protocol::types::mission_planning::DepthZeroReference::DEPTH_ZERO_REFERENCE_SURFACE
        );
        
        auto depth_command = blueye::protocol::DepthSetPointCommand(depth_set_point);
        
        // Send command to drone
        drone_.execute_command(depth_command);
        */
        
        return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error setting depth: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace blueye_bt_real