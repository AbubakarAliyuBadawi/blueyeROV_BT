#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs/srv/add_waypoint.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"
#include "mundus_mir_msgs/srv/go_to_waypoints.hpp"
#include "mundus_mir_msgs/srv/get_waypoint_status.hpp"
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"
#include <chrono>

// Forward declaration of global node
extern rclcpp::Node::SharedPtr g_node;

class NavigateToWaypoint : public BT::StatefulActionNode {
public:
    NavigateToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), first_run_(true), execution_started_(false), controller_enabled_(false)
    {
        clear_client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
        add_client_ = g_node->create_client<mundus_mir_msgs::srv::AddWaypoint>("/blueye/add_waypoint");
        run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>("/blueye/run_waypoint_controller");
        go_client_ = g_node->create_client<mundus_mir_msgs::srv::GoToWaypoints>("/blueye/go_to_waypoints");
        status_client_ = g_node->create_client<mundus_mir_msgs::srv::GetWaypointStatus>("/blueye/get_waypoint_status");
    }

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<double>("x", "X coordinate"),
            BT::InputPort<double>("y", "Y coordinate"),
            BT::InputPort<double>("z", "Z coordinate"),
            BT::InputPort<double>("velocity", "Desired velocity"),
            BT::InputPort<bool>("fixed_heading"),
            BT::InputPort<double>("heading")
        };
    }

protected:
    virtual BT::NodeStatus onStart() override;
    virtual BT::NodeStatus onRunning() override;
    virtual void onHalted() override;

private:
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_client_;
    rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr add_client_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;
    rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr go_client_;
    rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr status_client_;
    bool first_run_;
    bool execution_started_;
    bool controller_enabled_;

    bool enableController();
    bool clearWaypoints();
    bool addWaypoint(double x, double y, double z, double velocity);
    bool startWaypointController();
    bool startWaypointExecution();
    void stopWaypointController();
    void cleanup();
};