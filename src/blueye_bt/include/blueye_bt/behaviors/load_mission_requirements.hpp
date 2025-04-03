#ifndef LOAD_MISSION_REQUIREMENTS_HPP
#define LOAD_MISSION_REQUIREMENTS_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

/**
 * @brief Loads mission requirements from YAML file and sets them in the blackboard
 * 
 * This action node reads the mission requirements for a specific phase from the
 * mission_params.yaml file and sets the corresponding values in the blackboard.
 */
class LoadMissionRequirements : public BT::SyncActionNode
{
public:
    /**
     * @brief Constructor for LoadMissionRequirements
     * 
     * @param name Name of the node
     * @param config Node configuration
     */
    LoadMissionRequirements(const std::string& name, const BT::NodeConfiguration& config);

    /**
     * @brief Gets the static ports of this node
     * 
     * @return BT::PortsList List of ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Executes the action
     * 
     * Loads the mission requirements for the specified phase from the YAML file
     * and sets them in the blackboard.
     * 
     * @return BT::NodeStatus Status of the action (Success or Failure)
     */
    BT::NodeStatus tick() override;

private:
    // Parameters file path
    std::string params_file_path_;
};

#endif // LOAD_MISSION_REQUIREMENTS_HPP