#ifndef BLACKBOARD_CONDITION_HPP
#define BLACKBOARD_CONDITION_HPP

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class CheckBlackboard : public BT::ConditionNode {
private:
    // No additional members needed
public:
    CheckBlackboard(const std::string& name, const BT::NodeConfiguration& config);
    
    // Define the input ports
    static BT::PortsList providedPorts();
    
    // Override the tick method
    BT::NodeStatus tick() override;
};

class SetBlackboard : public BT::ConditionNode {  // Using ConditionNode for now since we know it exists
public:
    SetBlackboard(const std::string& name, const BT::NodeConfiguration& config);
    
    // Define the input ports
    static BT::PortsList providedPorts();
    
    // Override the tick method
    BT::NodeStatus tick() override;
};

#endif // BLACKBOARD_CONDITION_HPP