#ifndef BLUEYE_BT_REAL_NAVIGATE_WAYPOINT_ACTION_HPP
#define BLUEYE_BT_REAL_NAVIGATE_WAYPOINT_ACTION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <blueye/sdk/drone.hpp>
#include <chrono>

namespace blueye_bt_real {

class NavigateWaypointAction : public BT::StatefulActionNode {
private:
    // Blueye SDK drone reference
    blueye::sdk::Drone& drone_;
    
    // Navigation parameters
    double latitude_;
    double longitude_;
    double depth_;
    double speed_;
    
    // ROS logger
    rclcpp::Logger logger_;

public:
    NavigateWaypointAction(const std::string& name, const BT::NodeConfiguration& config, blueye::sdk::Drone& drone);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_NAVIGATE_WAYPOINT_ACTION_HPP