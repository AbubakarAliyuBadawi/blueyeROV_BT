// include/blueye_bt_real/conditions/CheckBlackboardBool.hpp
#pragma once

#include "behaviortree_cpp/condition_node.h"

namespace blueye_bt_real
{
class CheckBlackboardBool : public BT::ConditionNode
{
public:
    CheckBlackboardBool(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { 
            BT::InputPort<std::string>("key", "Key to check in blackboard"),
            BT::InputPort<bool>("expected_value", true, "Expected value"),
            BT::InputPort<bool>("invert_result", false, "Invert the result of the check")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string key;
        if (!getInput("key", key)) {
            throw BT::RuntimeError("Missing required input [key]");
        }

        bool expected_value = true;
        getInput("expected_value", expected_value);

        bool invert_result = false;
        getInput("invert_result", invert_result);

        // Try to get the value directly - older API style
        bool value = false;
        bool key_found = false;

        try {
            // This will throw if key doesn't exist
            value = config().blackboard->get<bool>(key);
            key_found = true;
        }
        catch (const std::exception& e) {
            key_found = false;
        }

        if (!key_found) {
            // Key not found, return failure
            return invert_result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        bool result = (value == expected_value);
        return (result != invert_result) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
}