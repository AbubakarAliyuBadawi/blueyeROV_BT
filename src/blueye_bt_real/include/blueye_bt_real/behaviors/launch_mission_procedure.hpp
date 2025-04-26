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
            BT::InputPort<double>("start_lat", "Starting latitude"),
            BT::InputPort<double>("start_lon", "Starting longitude"),
            BT::InputPort<double>("start_heading", "Starting heading in degrees")
        };
    }

    BT::NodeStatus tick() override;
};

#endif // LAUNCH_MISSION_PROCEDURE_HPP