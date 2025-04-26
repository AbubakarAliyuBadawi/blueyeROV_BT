// launch_docking_procedure.hpp
#ifndef LAUNCH_DOCKING_PROCEDURE_HPP
#define LAUNCH_DOCKING_PROCEDURE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class LaunchDockingProcedure : public BT::SyncActionNode {
public:
    LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus tick() override;
};

#endif // LAUNCH_DOCKING_PROCEDURE_HPP