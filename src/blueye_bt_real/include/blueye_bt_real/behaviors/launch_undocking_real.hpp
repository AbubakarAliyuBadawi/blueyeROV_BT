#ifndef LAUNCH_UNDOCKING_PROCEDURE_HPP
#define LAUNCH_UNDOCKING_PROCEDURE_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class LaunchUndockingProcedure : public BT::SyncActionNode
{
public:
    LaunchUndockingProcedure(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("script_path", "Path to the undocking script"),
            BT::InputPort<std::string>("drone_ip", "IP address of the drone"),
            BT::InputPort<int>("reverse_duration", "Duration to move backwards in seconds"),
            BT::InputPort<float>("reverse_power", "Power for backwards movement (0.1 to 1.0)")
        };
    }
    
    BT::NodeStatus tick() override;
};

#endif // LAUNCH_UNDOCKING_PROCEDURE_HPP