#ifndef PUBLISH_STATE_HPP
#define PUBLISH_STATE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

extern int g_current_mission_state;

// Forward declaration of global publisher
extern rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_mission_state_pub;

class PublishState : public BT::SyncActionNode {
public:
    PublishState(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
};

#endif // PUBLISH_STATE_HPP