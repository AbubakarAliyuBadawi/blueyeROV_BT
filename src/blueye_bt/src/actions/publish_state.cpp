#include "blueye_bt/actions/publish_state.hpp"

PublishState::PublishState(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
}

BT::PortsList PublishState::providedPorts() {
    return { BT::InputPort<int>("state", "Mission state ID") };
}

BT::NodeStatus PublishState::tick() {
    int state;
    if (!getInput<int>("state", state)) {
        RCLCPP_ERROR(rclcpp::get_logger("publish_state"), "Missing 'state' input");
        return BT::NodeStatus::FAILURE;
    }
    
    // Make sure the publisher exists
    if (!g_mission_state_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("publish_state"), "Mission state publisher not initialized");
        return BT::NodeStatus::FAILURE;
    }
    
    auto msg = std::make_unique<std_msgs::msg::Int32>();
    msg->data = state;
    g_mission_state_pub->publish(*msg);

    g_current_mission_state = state;
    
    // Log the state change
    const char* state_name = "Unknown";
    switch(state) {
        case 1: state_name = "Undocking"; break;
        case 2: state_name = "Transit_1"; break;
        case 3: state_name = "Pipeline Inspection"; break;
        case 4: state_name = "Transit_2"; break;
        case 5: state_name = "Wreckage Inspection"; break;
        case 6: state_name = "Homing"; break;
        case 7: state_name = "Docking"; break;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("publish_state"), "Mission state changed to: %d (%s)", state, state_name);
    return BT::NodeStatus::SUCCESS;
}