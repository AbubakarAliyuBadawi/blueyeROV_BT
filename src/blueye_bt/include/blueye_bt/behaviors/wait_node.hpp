// wait_node.hpp
#ifndef WAIT_NODE_HPP
#define WAIT_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

class Wait : public BT::StatefulActionNode {
public:
    Wait(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("duration", "Wait duration in seconds") };
    }
    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    
private:
    rclcpp::Time start_time_;
    int duration_seconds_;
};

#endif // WAIT_NODE_HPP