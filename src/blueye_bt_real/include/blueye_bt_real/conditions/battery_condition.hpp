#ifndef BLUEYE_BT_REAL_BATTERY_CONDITION_HPP
#define BLUEYE_BT_REAL_BATTERY_CONDITION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <blueye/sdk/drone.hpp>

namespace blueye_bt_real {

class BatteryCondition : public BT::ConditionNode {
private:
    // Blueye SDK drone reference
    blueye::sdk::Drone& drone_;
    
    // Battery threshold
    double threshold_;
    
    // ROS logger
    rclcpp::Logger logger_;

public:
    BatteryCondition(const std::string& name, const BT::NodeConfiguration& config, blueye::sdk::Drone& drone);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_BATTERY_CONDITION_HPP