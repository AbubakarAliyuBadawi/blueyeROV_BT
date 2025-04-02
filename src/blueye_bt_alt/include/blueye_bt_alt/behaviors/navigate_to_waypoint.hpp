#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs_alt/srv/add_waypoint_alt.hpp"
#include "mundus_mir_msgs_alt/srv/run_waypoint_controller_alt.hpp"
#include "mundus_mir_msgs_alt/srv/go_to_waypoints_alt.hpp"
#include "mundus_mir_msgs_alt/srv/get_waypoint_status_alt.hpp"
#include "mundus_mir_msgs_alt/srv/clear_waypoints_alt.hpp"
#include <chrono>
// Forward declaration of global node
extern rclcpp::Node::SharedPtr g_node;
class NavigateToWaypoint : public BT::StatefulActionNode {
public:
NavigateToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
 : BT::StatefulActionNode(name, config), first_run_(true), execution_started_(false), controller_enabled_(false)
 {
clear_client_ = g_node->create_client<mundus_mir_msgs_alt::srv::ClearWaypointsAlt>("/blueye/clear_waypoints");
add_client_ = g_node->create_client<mundus_mir_msgs_alt::srv::AddWaypointAlt>("/blueye/add_waypoint");
run_client_ = g_node->create_client<mundus_mir_msgs_alt::srv::RunWaypointControllerAlt>("/blueye/run_waypoint_controller");
go_client_ = g_node->create_client<mundus_mir_msgs_alt::srv::GoToWaypointsAlt>("/blueye/go_to_waypoints");
status_client_ = g_node->create_client<mundus_mir_msgs_alt::srv::GetWaypointStatusAlt>("/blueye/get_waypoint_status");
 }
static BT::PortsList providedPorts() {
return {
BT::InputPort<double>("x", "X coordinate"),
BT::InputPort<double>("y", "Y coordinate"),
BT::InputPort<double>("z", "Z coordinate"),
BT::InputPort<double>("velocity", 0.2, "Desired velocity"),
BT::InputPort<bool>("fixed_heading", false, "Whether to use fixed heading"),
BT::InputPort<double>("heading", 0.0, "Desired heading in radians"),
BT::InputPort<bool>("altitude_mode", false, "Whether to use altitude mode instead of fixed z"),
BT::InputPort<double>("target_altitude", 2.0, "Target altitude above seafloor in meters")
 };
 }
protected:
virtual BT::NodeStatus onStart() override;
virtual BT::NodeStatus onRunning() override;
virtual void onHalted() override;
private:
rclcpp::Client<mundus_mir_msgs_alt::srv::ClearWaypointsAlt>::SharedPtr clear_client_;
rclcpp::Client<mundus_mir_msgs_alt::srv::AddWaypointAlt>::SharedPtr add_client_;
rclcpp::Client<mundus_mir_msgs_alt::srv::RunWaypointControllerAlt>::SharedPtr run_client_;
rclcpp::Client<mundus_mir_msgs_alt::srv::GoToWaypointsAlt>::SharedPtr go_client_;
rclcpp::Client<mundus_mir_msgs_alt::srv::GetWaypointStatusAlt>::SharedPtr status_client_;
bool first_run_;
bool execution_started_;
bool controller_enabled_;
bool enableController();
bool clearWaypoints();
bool addWaypoint(double x, double y, double z, double velocity, bool fixed_heading, double heading,
bool altitude_mode, double target_altitude);
bool startWaypointController();
bool startWaypointExecution();
void stopWaypointController();
void cleanup();
};