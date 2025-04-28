#ifndef GOTO_WAYPOINT_CC_HPP
#define GOTO_WAYPOINT_CC_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class GoToWaypointCC : public BT::SyncActionNode
{
public:
    GoToWaypointCC(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("drone_ip", "192.168.1.101", "IP address of the drone"),
            BT::InputPort<double>("waypoint_lat", "Target latitude"),
            BT::InputPort<double>("waypoint_lon", "Target longitude"),
            BT::InputPort<double>("depth", 1.0, "Target depth in meters"),
            BT::InputPort<bool>("initialize_connection", false, "Whether to initialize a new connection")
        };
    }
    
    BT::NodeStatus tick() override;
};

#endif // GOTO_WAYPOINT_CC_HPP