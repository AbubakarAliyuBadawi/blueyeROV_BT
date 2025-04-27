#ifndef LAUNCH_SIMPLE_DEPTH_MISSION_HPP
#define LAUNCH_SIMPLE_DEPTH_MISSION_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class LaunchSimpleDepthMission : public BT::SyncActionNode
{
public:
    LaunchSimpleDepthMission(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return { };  // No parameters needed for simple version
    }
    
    BT::NodeStatus tick() override;
};

#endif // LAUNCH_SIMPLE_DEPTH_MISSION_HPP