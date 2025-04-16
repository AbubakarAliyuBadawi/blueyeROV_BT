#ifndef ABORT_ON_CONDITION_DECORATOR_HPP
#define ABORT_ON_CONDITION_DECORATOR_HPP

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class AbortOnCondition : public BT::DecoratorNode {
public:
    AbortOnCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config)
    {
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("condition", "Blackboard key to monitor for abortion")
        };
    }

    BT::NodeStatus tick() override {
        // Get the blackboard condition key
        std::string condition_key;
        if (!getInput("condition", condition_key)) {
            return BT::NodeStatus::FAILURE;
        }

        // Check blackboard value - looking for "true"
        std::string value;
        if (getInput(condition_key, value) && value == "true") {
            // Condition is true, abort execution
            return BT::NodeStatus::FAILURE;
        }

        // If we're here, condition is false, so execute child
        const BT::NodeStatus child_status = child()->executeTick();
        
        // Check again after child execution (in case the child took a while)
        if (getInput(condition_key, value) && value == "true") {
            // Condition became true during execution, abort
            haltChild();
            return BT::NodeStatus::FAILURE;
        }

        return child_status;
    }

private:
    void haltChild() {
        const auto& child_node = child();
        if (child_node->status() == BT::NodeStatus::RUNNING) {
            haltChild();
        }
    }
};

#endif // ABORT_ON_CONDITION_DECORATOR_HPP