#include "rov_mission_bt/mission_control/mission_loader.hpp"
#include "rov_mission_bt/mission_control/mission_factory.hpp"
#include "rov_mission_bt/utils/node_configuration.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    auto node = rov_mission_bt::initializeNode(argc, argv);
    
    // Initialize mission factory and register behaviors
    rov_mission_bt::MissionFactory factory;
    factory.registerBehaviors();
    
    // Initialize mission loader
    std::string mission_dir = node->get_parameter("mission_directory").as_string();
    rov_mission_bt::MissionLoader loader(mission_dir);
    
    // Get mission name from parameter
    std::string mission_name = node->get_parameter("mission_name").as_string();
    
    try {
        // Load and execute mission
        auto tree = loader.loadMission(mission_name);
        
        // Mission execution loop
        while (rclcpp::ok()) {
            auto status = tree.tickRoot();
            // ... rest of execution logic
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Mission execution failed: %s", e.what());
        return 1;
    }
    
    return 0;
}