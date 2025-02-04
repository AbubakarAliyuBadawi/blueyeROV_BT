#ifndef ROV_MISSION_BT_WAYPOINT_BEHAVIORS_HPP
#define ROV_MISSION_BT_WAYPOINT_BEHAVIORS_HPP

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include "mundus_mir_msgs/srv/add_waypoint.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"
#include "mundus_mir_msgs/srv/go_to_waypoints.hpp"
#include "mundus_mir_msgs/srv/get_waypoint_status.hpp"
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"

// Forward declaration of global node
extern rclcpp::Node::SharedPtr g_node;

class ClearWaypoints : public BT::SyncActionNode {
public:
    ClearWaypoints(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr client_;
};

class SetWaypoint : public BT::SyncActionNode {
public:
    SetWaypoint(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr client_;
};

class ExecuteWaypoint : public BT::StatefulActionNode {
public:
    ExecuteWaypoint(const std::string& name, const BT::NodeConfiguration& config);
    ~ExecuteWaypoint();
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;
    rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr go_client_;
    rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr status_client_;
    bool first_run_;
    bool execution_started_;

    bool startWaypointController();
    bool startWaypointExecution();
    void stopWaypointController();
    void cleanup();
};

#endif // ROV_MISSION_BT_WAYPOINT_BEHAVIORS_HPP