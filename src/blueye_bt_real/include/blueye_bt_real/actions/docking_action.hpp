#ifndef BLUEYE_BT_REAL_DOCKING_ACTION_HPP
#define BLUEYE_BT_REAL_DOCKING_ACTION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
// Include Blueye SDK headers here
// #include <blueye/sdk/drone.hpp>

namespace blueye_bt_real {

class DockingAction : public BT::StatefulActionNode {
private:
    // Blueye SDK drone reference
    // blueye::sdk::Drone& drone_;
    
    // Docking parameters
    double latitude_;
    double longitude_;
    double depth_;
    double approach_speed_;
    double descent_speed_;
    double acceptance_radius_;
    
    // Timing for monitoring progress
    std::chrono::steady_clock::time_point start_time_;
    long last_logged_time_ = -1;
    
    // ROS logger
    rclcpp::Logger logger_;

public:
    DockingAction(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_DOCKING_ACTION_HPP