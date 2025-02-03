#pragma once
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>

namespace rov_mission_bt {

class BehaviorBase {
protected:
    // Shared functionality for all behaviors
    bool waitForService(rclcpp::ClientBase::SharedPtr client, 
                       const std::string& service_name,
                       const rclcpp::Logger& logger);
    
    // Common error handling
    BT::NodeStatus handleServiceError(const std::string& service_name,
                                    const rclcpp::Logger& logger);
};

} // namespace rov_mission_bt