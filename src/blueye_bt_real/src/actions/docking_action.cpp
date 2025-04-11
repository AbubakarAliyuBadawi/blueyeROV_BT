#include "blueye_bt_real/actions/docking_action.hpp"

namespace blueye_bt_real {

DockingAction::DockingAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config),
      logger_(rclcpp::get_logger("docking_action"))
{
    // Initialize SDK drone reference here if needed
}

BT::PortsList DockingAction::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("latitude", "Docking station latitude in degrees"));
    ports.insert(BT::InputPort<double>("longitude", "Docking station longitude in degrees"));
    ports.insert(BT::InputPort<double>("depth", "Docking depth in meters"));
    ports.insert(BT::InputPort<double>("approach_speed", "Final approach speed in m/s"));
    ports.insert(BT::InputPort<double>("descent_speed", "Vertical speed in m/s"));
    ports.insert(BT::InputPort<double>("acceptance_radius", "Circle of acceptance radius in meters"));
    return ports;
}

BT::NodeStatus DockingAction::onStart()
{
    // Get parameters from ports
    if (!getInput("latitude", latitude_) || !getInput("longitude", longitude_)) {
        RCLCPP_ERROR(logger_, "Missing required inputs for docking");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get optional parameters with defaults
    if (!getInput("depth", depth_)) {
        depth_ = 2.3;  // Default docking depth if not provided
    }
    
    if (!getInput("approach_speed", approach_speed_)) {
        approach_speed_ = 0.2;  // Default approach speed if not provided
    }
    
    if (!getInput("descent_speed", descent_speed_)) {
        descent_speed_ = 0.2;  // Default descent speed if not provided
    }
    
    if (!getInput("acceptance_radius", acceptance_radius_)) {
        acceptance_radius_ = 0.5;  // Default acceptance radius if not provided
    }
    
    RCLCPP_INFO(logger_, "Starting docking sequence: lat=%.6f, lon=%.6f, depth=%.2f",
               latitude_, longitude_, depth_);
    
    try {
        // Here we would create and execute a special docking mission using Blueye SDK
        /*
        // Example SDK code (replace with actual SDK calls)
        
        // Create mission instructions for docking
        std::vector<blueye::protocol::Instruction> instructions;
        int instruction_id = 1;
        
        // Step 1: Configure auto-depth mode
        auto control_mode = blueye::protocol::Instruction(
            instruction_id++,
            blueye::protocol::ControlModeCommand(
                blueye::protocol::ControlModeVertical::CONTROL_MODE_VERTICAL_AUTO_DEPTH,
                blueye::protocol::ControlModeHorizontal::CONTROL_MODE_HORIZONTAL_MANUAL
            ),
            true  // auto_continue
        );
        instructions.push_back(control_mode);
        
        // Step 2: Set docking depth (typically deeper than approach)
        auto docking_depth_set_point = blueye::protocol::DepthSetPoint(
            depth_,  // Docking depth
            descent_speed_,
            blueye::protocol::types::mission_planning::DepthZeroReference::DEPTH_ZERO_REFERENCE_SURFACE
        );
        
        auto goto_docking_depth = blueye::protocol::Instruction(
            instruction_id++,
            blueye::protocol::DepthSetPointCommand(
                docking_depth_set_point
            ),
            true  // auto_continue
        );
        instructions.push_back(goto_docking_depth);
        
        // Step 3: Navigate to docking station
        auto docking_waypoint = blueye::protocol::Waypoint(
            instruction_id,
            "Docking Station",
            blueye::protocol::types::message_formats::LatLongPosition(
                latitude_,
                longitude_
            ),
            acceptance_radius_,  // More precise for docking
            approach_speed_,     // Slower approach speed for docking
            docking_depth_set_point
        );
        
        auto goto_docking = blueye::protocol::Instruction(
            instruction_id,
            blueye::protocol::WaypointCommand(
                docking_waypoint
            ),
            true  // auto_continue
        );
        instructions.push_back(goto_docking);
        
        // Create and send the docking mission
        auto mission = blueye::protocol::Mission(
            1,
            "Docking Mission",
            instructions,
            approach_speed_,  // default_surge_speed
            descent_speed_,   // default_heave_speed
            acceptance_radius_  // default_circle_of_acceptance
        );
        
        // Clear any existing missions
        drone_.mission.clear();
        
        // Send mission to drone
        drone_.mission.send_new(mission);
        
        // Start mission execution
        drone_.mission.run();
        */
        
        // For demonstration purposes
        RCLCPP_INFO(logger_, "Docking mission created and started");
        
        // Record mission start time for tracking
        start_time_ = std::chrono::steady_clock::now();
        
        return BT::NodeStatus::RUNNING;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error starting docking: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus DockingAction::onRunning()
{
    try {
        // Here you would check mission status
        /*
        // Example SDK code (replace with actual SDK calls)
        auto status = drone_.mission.get_status();
        
        // Log telemetry
        RCLCPP_INFO(logger_, 
                  "Docking: %s, Progress: %d/%d, Depth: %.1f",
                  status.state.name.c_str(),
                  status.completed_instruction_ids.size(),
                  status.total_number_of_instructions,
                  drone_.depth);
        
        // Check if mission is complete
        if (status.state == blueye::protocol::MissionState::MISSION_STATE_COMPLETED) {
            RCLCPP_INFO(logger_, "Docking completed successfully");
            return BT::NodeStatus::SUCCESS;
        }
        else if (status.state == blueye::protocol::MissionState::MISSION_STATE_ABORTED ||
                status.state == blueye::protocol::MissionState::MISSION_STATE_FAILED_TO_LOAD_MISSION ||
                status.state == blueye::protocol::MissionState::MISSION_STATE_FAILED_TO_START_MISSION) {
            RCLCPP_ERROR(logger_, "Docking failed: %s", status.state.name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        */
        
        // For demonstration purposes without SDK
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time_).count();
        
        // Log periodic updates
        if (elapsed_seconds % 5 == 0 && elapsed_seconds != last_logged_time_) {
            RCLCPP_INFO(logger_, "Docking in progress: %ld seconds elapsed", elapsed_seconds);
            last_logged_time_ = elapsed_seconds;
        }
        
        // Simulate docking completion after 30 seconds
        if (elapsed_seconds >= 30) {
            RCLCPP_INFO(logger_, "Docking completed successfully");
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error during docking: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

void DockingAction::onHalted()
{
    RCLCPP_INFO(logger_, "Halting docking sequence");
    
    try {
        // Here you would stop mission execution
        /*
        // Example SDK code (replace with actual SDK calls)
        drone_.mission.stop();
        */
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error stopping docking: %s", e.what());
    }
}

}  // namespace blueye_bt_real