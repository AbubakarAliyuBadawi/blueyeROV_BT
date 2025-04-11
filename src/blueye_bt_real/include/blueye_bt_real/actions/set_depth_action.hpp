#ifndef BLUEYE_BT_REAL_SET_DEPTH_ACTION_HPP
#define BLUEYE_BT_REAL_SET_DEPTH_ACTION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
// Include Blueye SDK headers here
// #include <blueye/sdk/drone.hpp>

namespace blueye_bt_real {

class SetDepthAction : public BT::SyncActionNode {
private:
    // Blueye SDK drone reference
    // blueye::sdk::Drone& drone_;
    
    // ROS logger
    rclcpp::Logger logger_;

public:
    SetDepthAction(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_SET_DEPTH_ACTION_HPP