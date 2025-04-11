#include "blueye_bt_real/actions/navigate_waypoint_action.hpp"
#include <blueye/protocol/mission.hpp>
#include <blueye/protocol/types/mission_planning.hpp>
#include <blueye/protocol/types/message_formats.hpp>

namespace blueye_bt_real {

NavigateWaypointAction::NavigateWaypointAction(const std::string& name, const BT::NodeConfiguration& config, blueye::sdk::Drone& drone)
    : BT::StatefulActionNode(name, config),
      drone_(drone),
      logger_(rclcpp::get_logger("navigate_waypoint_action"))
{
}

BT::PortsList NavigateWaypointAction::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("latitude", "Waypoint latitude in degrees"));
    ports.insert(BT::InputPort<double>("longitude", "Waypoint longitude in degrees"));
    ports.insert(BT::InputPort<double>("depth", "Target depth in meters"));
    ports.insert(BT::InputPort<double>("speed", "Navigation speed in m/s"));
    return ports;
}

BT::NodeStatus NavigateWaypointAction::onStart()
{
    // Get parameters from ports
    if (!getInput("latitude", latitude_) || !getInput("longitude", longitude_) || 
        !getInput("depth", depth_)) {
        RCLCPP_ERROR(logger_, "Missing required inputs for waypoint navigation");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get optional speed parameter
    if (!getInput("speed", speed_)) {
        speed_ = 0.3;  // Default speed if not provided
    }
    
    RCLCPP_INFO(logger_, "Starting navigation to waypoint: lat=%.6f, lon=%.6f, depth=%.2f, speed=%.2f",
               latitude_, longitude_, depth_, speed_);
    
    try {
        // 1. Clear any existing missions
        drone_.mission.clear();
        
        // 2. Create depth set point
        auto depth_set_point = blueye::protocol::DepthSetPoint(
            depth_,
            0.2,  // speed_to_depth
            blueye::protocol::types::mission_planning::DepthZeroReference::DEPTH_ZERO_REFERENCE_SURFACE
        );
        
        // 3. Create instructions vector
        std::vector<blueye::protocol::Instruction> instructions;
        int instruction_id = 1;
        
        // 4. Add control mode instruction
        auto control_mode = blueye::protocol::Instruction(
            instruction_id++,
            blueye::protocol::ControlModeCommand(
                blueye::protocol::ControlModeVertical::CONTROL_MODE_VERTICAL_AUTO_DEPTH,
                blueye::protocol::ControlModeHorizontal::CONTROL_MODE_HORIZONTAL_MANUAL
            ),
            true  // auto_continue
        );
        instructions.push_back(control_mode);
        
        // 5. Add depth set point instruction
        auto goto_depth = blueye::protocol::Instruction(
            instruction_id++,
            blueye::protocol::DepthSetPointCommand(
                depth_set_point
            ),
            true  // auto_continue
        );
        instructions.push_back(goto_depth);
        
        // 6. Create waypoint
        auto waypoint = blueye::protocol::Waypoint(
            instruction_id,
            "Target Waypoint",
            blueye::protocol::types::message_formats::LatLongPosition(
                latitude_,
                longitude_
            ),
            1.0,  // circle_of_acceptance
            speed_,
            depth_set_point
        );
        
        // 7. Add waypoint instruction
        auto goto_waypoint = blueye::protocol::Instruction(
            instruction_id,
            blueye::protocol::WaypointCommand(
                waypoint
            ),
            true  // auto_continue
        );
        instructions.push_back(goto_waypoint);
        
        // 8. Create mission
        auto mission = blueye::protocol::Mission(
            1,  // id
            "Waypoint Mission",
            instructions,
            speed_,  // default_surge_speed
            0.2,     // default_heave_speed
            1.0      // default_circle_of_acceptance
        );
        
        // 9. Send mission to drone
        drone_.mission.send_new(mission);
        
        // 10. Start mission execution
        drone_.mission.run();
        
        return BT::NodeStatus::RUNNING;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error starting navigation: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus NavigateWaypointAction::onRunning()
{
    try {
        // Check mission progress
        auto status = drone_.mission.get_status();
        
        // Log current status
        std::string state_msg = "Mission: " + std::string(status.state.name) + ", ";
        state_msg += "Progress: " + std::to_string(status.completed_instruction_ids.size()) + "/" + 
                     std::to_string(status.total_number_of_instructions) + " instructions, ";
        state_msg += "Time: " + std::to_string(status.time_elapsed) + "s/" + 
                     std::to_string(status.time_elapsed + status.estimated_time_to_complete) + "s";
                     
        try {
            state_msg += ", Depth: " + std::to_string(drone_.depth) + "m";
        } catch (...) {
            // Ignore depth errors
        }
        
        RCLCPP_INFO(logger_, "%s", state_msg.c_str());
        
        // Check mission state
        if (status.state == blueye::protocol::MissionState::MISSION_STATE_COMPLETED) {
            RCLCPP_INFO(logger_, "Waypoint navigation completed successfully");
            return BT::NodeStatus::SUCCESS;
        }
        else if (status.state == blueye::protocol::MissionState::MISSION_STATE_ABORTED) {
            RCLCPP_WARNING(logger_, "Mission was aborted");
            return BT::NodeStatus::FAILURE;
        }
        else if (status.state == blueye::protocol::MissionState::MISSION_STATE_FAILED_TO_LOAD_MISSION ||
                status.state == blueye::protocol::MissionState::MISSION_STATE_FAILED_TO_START_MISSION) {
            RCLCPP_ERROR(logger_, "Mission failed with state: %s", status.state.name);
            return BT::NodeStatus::FAILURE;
        }
        
        return BT::NodeStatus::RUNNING;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error during navigation: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

void NavigateWaypointAction::onHalted()
{
    RCLCPP_INFO(logger_, "Halting waypoint navigation");
    
    try {
        // Stop mission
        drone_.mission.stop();
        RCLCPP_INFO(logger_, "Mission stopped");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error stopping navigation: %s", e.what());
    }
}

}  // namespace blueye_bt_real