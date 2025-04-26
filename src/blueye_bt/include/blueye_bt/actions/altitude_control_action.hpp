#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_srvs/srv/set_bool.hpp>

// Forward declaration
extern rclcpp::Node::SharedPtr g_node;

class AltitudeControlAction : public BT::SyncActionNode {
public:
    AltitudeControlAction(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<bool>("enable", true, "Enable altitude control"),
            BT::InputPort<double>("target_altitude", 2.0, "Target altitude in meters")
        };
    }
    
    BT::NodeStatus tick() override;
    
private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
    std::shared_ptr<rclcpp::SyncParametersClient> param_client_;
};