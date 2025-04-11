#include "blueye_bt/conditions/blackboard_condition.hpp"

CheckBlackboard::CheckBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList CheckBlackboard::providedPorts() {
    return {
        BT::InputPort<std::string>("key", "Key to check in the blackboard"),
        BT::InputPort<std::string>("expected_value", "Expected value to compare with")
    };
}

BT::NodeStatus CheckBlackboard::tick() {
    // Get the key to check
    std::string key;
    if (!getInput<std::string>("key", key)) {
        RCLCPP_ERROR(rclcpp::get_logger("check_blackboard"), "Missing required input [key]");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get the expected value
    std::string expected_value;
    if (!getInput<std::string>("expected_value", expected_value)) {
        RCLCPP_ERROR(rclcpp::get_logger("check_blackboard"), "Missing required input [expected_value]");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get the actual value from the blackboard
    std::string actual_value;
    if (!config().blackboard->get(key, actual_value)) {
        RCLCPP_DEBUG(rclcpp::get_logger("check_blackboard"), 
                    "Key [%s] not found in blackboard", key.c_str());
        return BT::NodeStatus::FAILURE;
    }
    
    // Compare the values
    bool match = (actual_value == expected_value);
    RCLCPP_DEBUG(rclcpp::get_logger("check_blackboard"), 
                "Comparing blackboard key [%s]: expected [%s], actual [%s], match=%s",
                key.c_str(), expected_value.c_str(), actual_value.c_str(), 
                match ? "true" : "false");
    
    return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

SetBlackboard::SetBlackboard(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {
}

BT::PortsList SetBlackboard::providedPorts() {
    return {
        BT::InputPort<std::string>("output_key", "Key to set in the blackboard"),
        BT::InputPort<std::string>("value", "Value to set")
    };
}

BT::NodeStatus SetBlackboard::tick() {
    // Get the key to set
    std::string output_key;
    if (!getInput<std::string>("output_key", output_key)) {
        RCLCPP_ERROR(rclcpp::get_logger("set_blackboard"), "Missing required input [output_key]");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get the value to set
    std::string value;
    if (!getInput<std::string>("value", value)) {
        RCLCPP_ERROR(rclcpp::get_logger("set_blackboard"), "Missing required input [value]");
        return BT::NodeStatus::FAILURE;
    }
    
    // Set the value in the blackboard
    config().blackboard->set(output_key, value);
    RCLCPP_DEBUG(rclcpp::get_logger("set_blackboard"), 
                "Setting blackboard key [%s] to value [%s]",
                output_key.c_str(), value.c_str());
    
    // Even though this is a ConditionNode, we're using it as an action,
    // so always return SUCCESS if we got here
    return BT::NodeStatus::SUCCESS;
}