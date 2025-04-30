// PipelineInspectionMission.hpp
#ifndef PIPELINE_INSPECTION_MISSION_HPP
#define PIPELINE_INSPECTION_MISSION_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class PipelineInspectionMission : public BT::SyncActionNode
{
public:
    PipelineInspectionMission(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("drone_ip", "192.168.1.101", "IP address of the drone"),
            BT::InputPort<bool>("goto_docking", true, "Whether to go to docking position at the end")
        };
    }
    
    BT::NodeStatus tick() override;
};

#endif // PIPELINE_INSPECTION_MISSION_HPP