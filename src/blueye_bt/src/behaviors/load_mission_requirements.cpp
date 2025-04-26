#include "blueye_bt/behaviors/load_mission_requirements.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

LoadMissionRequirements::LoadMissionRequirements(
    const std::string& name, 
    const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // Get the path to the configuration file
    std::string package_path = ament_index_cpp::get_package_share_directory("blueye_bt");
    params_file_path_ = package_path + "/config/mission_params.yaml";
}

BT::PortsList LoadMissionRequirements::providedPorts()
{
    // Define the input port for the mission phase
    return { BT::InputPort<std::string>("phase") };
}

BT::NodeStatus LoadMissionRequirements::tick()
{
    // Get the mission phase from the input port
    std::string phase;
    if (!getInput<std::string>("phase", phase)) {
        RCLCPP_ERROR(rclcpp::get_logger("load_mission_requirements"), "Missing 'phase' input");
        return BT::NodeStatus::FAILURE;
    }

    try {
        // Load the YAML file - renamed variable from 'config' to 'yaml_config'
        YAML::Node yaml_config = YAML::LoadFile(params_file_path_);
        
        // Check if mission_requirements and the specified phase exist
        if (!yaml_config["mission_requirements"] || !yaml_config["mission_requirements"][phase]) {
            RCLCPP_ERROR(rclcpp::get_logger("load_mission_requirements"), 
                       "Missing mission requirements for phase '%s'", phase.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // Get the requirements for this phase
        YAML::Node phase_config = yaml_config["mission_requirements"][phase];
        
        // Set the camera requirement in the blackboard
        bool requires_camera = phase_config["requires_camera"].as<bool>(false);
        config().blackboard->set("mission_requires_camera", requires_camera ? "true" : "false");
        
        // Set the sonar requirement in the blackboard
        bool requires_sonar = phase_config["requires_sonar"].as<bool>(false);
        config().blackboard->set("mission_requires_sonar", requires_sonar ? "true" : "false");
        
        RCLCPP_INFO(rclcpp::get_logger("load_mission_requirements"), 
                   "Loaded mission requirements for phase '%s': camera=%s, sonar=%s",
                   phase.c_str(),
                   (requires_camera ? "required" : "not required"),
                   (requires_sonar ? "required" : "not required"));
        
        return BT::NodeStatus::SUCCESS;
    }
    catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("load_mission_requirements"), 
                   "Error loading YAML file: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("load_mission_requirements"), 
                   "Error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}