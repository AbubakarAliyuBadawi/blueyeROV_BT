#ifndef LAUNCH_MISSION_PROCEDURE_HPP
#define LAUNCH_MISSION_PROCEDURE_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class LaunchMissionProcedure : public BT::SyncActionNode
{
public:
    LaunchMissionProcedure(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("script_path", "Path to the mission script"),
            BT::InputPort<std::string>("drone_ip", "IP address of the drone"),
            BT::InputPort<int>("max_retries", "Maximum number of retry attempts for aborted missions")
        };
    }
    
    BT::NodeStatus tick() override;
};

#endif // LAUNCH_MISSION_PROCEDURE_HPP