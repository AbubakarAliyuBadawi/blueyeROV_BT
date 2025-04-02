// include/blueye_bt_alt/behaviors/station_keeping.hpp
#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs_alt/srv/add_waypoint_alt.hpp"
#include "mundus_mir_msgs_alt/srv/run_waypoint_controller_alt.hpp"
#include "mundus_mir_msgs_alt/srv/go_to_waypoints_alt.hpp"
#include "mundus_mir_msgs_alt/srv/clear_waypoints_alt.hpp"
#include "mundus_mir_msgs_alt/srv/get_waypoint_status_alt.hpp"
#include <chrono>

// Forward declaration of global node
extern rclcpp::Node::SharedPtr g_node;

class StationKeeping : public BT::StatefulActionNode {
public:
    StationKeeping(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("duration", "Duration in seconds"),
            BT::InputPort<double>("heading", "Desired heading in degrees"),
            BT::InputPort<double>("x", "X coordinate for station keeping (optional)"),
            BT::InputPort<double>("y", "Y coordinate for station keeping (optional)"),
            BT::InputPort<double>("z", "Z coordinate for station keeping (optional)"),
            BT::InputPort<bool>("altitude_mode", false, "Whether to use altitude mode instead of fixed z"),
            BT::InputPort<double>("target_altitude", 2.0, "Target altitude above seafloor in meters")
        };
    }
    
protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    
private:
    // Time tracking
    std::chrono::steady_clock::time_point start_time_;
    
    // Service clients
    rclcpp::Client<mundus_mir_msgs_alt::srv::ClearWaypointsAlt>::SharedPtr clear_client_;
    rclcpp::Client<mundus_mir_msgs_alt::srv::AddWaypointAlt>::SharedPtr add_client_;
    rclcpp::Client<mundus_mir_msgs_alt::srv::RunWaypointControllerAlt>::SharedPtr run_client_;
    rclcpp::Client<mundus_mir_msgs_alt::srv::GoToWaypointsAlt>::SharedPtr go_client_;
    rclcpp::Client<mundus_mir_msgs_alt::srv::GetWaypointStatusAlt>::SharedPtr status_client_;
        
    // Helper methods
    bool clearWaypoints();
    bool addWaypoint(double heading, bool altitude_mode = false, double target_altitude = 2.0);
    bool startWaypointController(bool run);
    bool startWaypointExecution();
    void stopExecution();
};