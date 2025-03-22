// mission_control_main.cpp
#include "blueye_bt/managers/mission_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Parse command line arguments
    std::string mission_type = "none";
    bool force = false;
    
    if (argc > 1) {
        mission_type = argv[1];
    }
    
    if (argc > 2 && std::string(argv[2]) == "--force") {
        force = true;
    }
    
    if (mission_type != "pipeline" && mission_type != "lawnmower" && mission_type != "none") {
        print_usage();
        rclcpp::shutdown();
        return 1;
    }
    
    // Create the mission control client
    auto client = std::make_shared<MissionControlClient>();
    
    // Switch mission
    bool success = client->switch_mission(mission_type, force);
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}