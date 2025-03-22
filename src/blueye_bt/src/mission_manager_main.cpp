// mission_manager_main.cpp
#include "blueye_bt/managers/mission_manager.hpp"
#include <rclcpp/rclcpp.hpp>

// Define the global node pointer here
rclcpp::Node::SharedPtr g_node;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Create the mission manager as a shared_ptr
    auto mission_manager = std::make_shared<MissionManager>();
    
    // Spin the node
    rclcpp::spin(mission_manager);
    
    rclcpp::shutdown();
    return 0;
}