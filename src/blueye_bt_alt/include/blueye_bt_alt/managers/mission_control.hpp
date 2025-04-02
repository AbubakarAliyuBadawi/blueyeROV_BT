#ifndef MISSION_CONTROL_HPP
#define MISSION_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "mundus_mir_msgs/srv/mission_manager.hpp"

class MissionControlClient : public rclcpp::Node {
public:
    // Constructor
    MissionControlClient();
    
    // Request a mission switch
    bool switch_mission(const std::string& mission_type, bool force = false);
    
private:
    // ROS service client
    rclcpp::Client<mundus_mir_msgs::srv::MissionManager>::SharedPtr client_;
};

// Helper function to print usage information
void print_usage();

#endif // MISSION_CONTROL_HPP