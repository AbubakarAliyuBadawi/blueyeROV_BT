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
        return {
            BT::InputPort<std::string>("script_path", "Path to the depth mission script"),
            BT::InputPort<std::string>("drone_ip", "IP address of the drone"),
            BT::InputPort<double>("start_lat", "Starting latitude"),
            BT::InputPort<double>("start_lon", "Starting longitude"),
            BT::InputPort<double>("start_heading", "Starting heading in degrees"),
            BT::InputPort<double>("target_depth", "Target depth in meters"),
            BT::InputPort<int>("duration", "Duration to maintain depth in seconds"),
        };
    }
    
    BT::NodeStatus tick() override;
};

#endif // LAUNCH_SIMPLE_DEPTH_MISSION_HPP