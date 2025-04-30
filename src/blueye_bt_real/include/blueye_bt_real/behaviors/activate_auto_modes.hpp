#pragma once
#include <memory>
#include <string>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

// Change from SyncActionNode to StatefulActionNode
class ActivateAutoModes : public BT::StatefulActionNode
{
public:
    ActivateAutoModes(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("depth_hold", true, "Enable depth hold mode"),
            BT::InputPort<bool>("heading_hold", true, "Enable heading hold mode"),
            BT::InputPort<int>("hold_duration", 10, "Duration to hold position in seconds")
        };
    }

    // StatefulActionNode interface methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr depth_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_client_;
    
    bool callAutoService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool enable, const std::string& service_name);
    
    // New members for time tracking
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    int hold_duration_; // in seconds
    bool services_activated_;
};