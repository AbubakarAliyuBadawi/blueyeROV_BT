#ifndef BLUEYE_BT_REAL_MISSION_STATUS_CONDITION_HPP
#define BLUEYE_BT_REAL_MISSION_STATUS_CONDITION_HPP

#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace blueye_bt_real {

class MissionStatusCondition : public BT::ConditionNode {
private:
    // ROS logger
    rclcpp::Logger logger_;

public:
    MissionStatusCondition(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_MISSION_STATUS_CONDITION_HPP