#include "rov_mission_bt/behaviors/waypoint_behaviors.hpp"

// ClearWaypoints Implementation
ClearWaypoints::ClearWaypoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) 
{
    client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    RCLCPP_INFO(g_node->get_logger(), "Creating ClearWaypoints node");
}

BT::PortsList ClearWaypoints::providedPorts() {
    return BT::PortsList();
}

BT::NodeStatus ClearWaypoints::tick() {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(g_node->get_logger(), "Clear waypoints service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    auto future = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call clear waypoints service");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(g_node->get_logger(), "Successfully cleared waypoints");
    return BT::NodeStatus::SUCCESS;
}

// SetWaypoint Implementation
SetWaypoint::SetWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    client_ = g_node->create_client<mundus_mir_msgs::srv::AddWaypoint>("/blueye/add_waypoint");
}

BT::PortsList SetWaypoint::providedPorts() {
    return { BT::InputPort<double>("x"),
             BT::InputPort<double>("y"),
             BT::InputPort<double>("z"),
             BT::InputPort<double>("velocity") };
}

BT::NodeStatus SetWaypoint::tick() {
    double x, y, z, velocity;
    if (!getInput("x", x) || !getInput("y", y) || 
        !getInput("z", z) || !getInput("velocity", velocity)) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to get input parameters");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    request->x = x;
    request->y = y;
    request->z = z;
    request->desired_velocity = velocity;
    request->fixed_heading = false;
    request->heading = 0.0;

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(g_node->get_logger(), "Add waypoint service not available");
        return BT::NodeStatus::FAILURE;
    }

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call add waypoint service");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(g_node->get_logger(), "Successfully set waypoint: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    return BT::NodeStatus::SUCCESS;
}

// ExecuteWaypoint Implementation
ExecuteWaypoint::ExecuteWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), first_run_(true), execution_started_(false)
{
    run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>("/blueye/run_waypoint_controller");
    go_client_ = g_node->create_client<mundus_mir_msgs::srv::GoToWaypoints>("/blueye/go_to_waypoints");
    status_client_ = g_node->create_client<mundus_mir_msgs::srv::GetWaypointStatus>("/blueye/get_waypoint_status");
}

ExecuteWaypoint::~ExecuteWaypoint() {
    cleanup();
}

BT::PortsList ExecuteWaypoint::providedPorts() {
    return BT::PortsList();
}

BT::NodeStatus ExecuteWaypoint::onStart() {
    if (!first_run_) {
        return BT::NodeStatus::RUNNING;
    }

    // Start the waypoint controller
    if (!startWaypointController()) {
        return BT::NodeStatus::FAILURE;
    }

    // Start moving to waypoints
    if (!startWaypointExecution()) {
        stopWaypointController();
        return BT::NodeStatus::FAILURE;
    }

    first_run_ = false;
    execution_started_ = true;
    RCLCPP_INFO(g_node->get_logger(), "Started waypoint execution");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteWaypoint::onRunning() {
    auto status_request = std::make_shared<mundus_mir_msgs::srv::GetWaypointStatus::Request>();
    
    if (!status_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(g_node->get_logger(), "Get waypoint status service not available");
        return BT::NodeStatus::FAILURE;
    }
    
    auto future = status_client_->async_send_request(status_request);
    if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call get waypoint status service");
        return BT::NodeStatus::FAILURE;
    }

    auto result = future.get();
    if (result->status_code.find("Currently going to waypoint: 0") != std::string::npos) {
        RCLCPP_INFO(g_node->get_logger(), "Waypoint execution completed");
        cleanup();
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void ExecuteWaypoint::onHalted() {
    RCLCPP_INFO(g_node->get_logger(), "Halting waypoint execution");
    cleanup();
}

bool ExecuteWaypoint::startWaypointController() {
    auto run_request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    run_request->run = true;
    
    if (!run_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(g_node->get_logger(), "Run waypoint controller service not available");
        return false;
    }
    
    auto run_future = run_client_->async_send_request(run_request);
    if (rclcpp::spin_until_future_complete(g_node, run_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call run waypoint controller service");
        return false;
    }

    return true;
}

bool ExecuteWaypoint::startWaypointExecution() {
    auto go_request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
    go_request->run = true;
    
    if (!go_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(g_node->get_logger(), "Go to waypoints service not available");
        return false;
    }
    
    auto go_future = go_client_->async_send_request(go_request);
    if (rclcpp::spin_until_future_complete(g_node, go_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call go to waypoints service");
        return false;
    }

    return true;
}

void ExecuteWaypoint::stopWaypointController() {
    if (!execution_started_) return;

    auto run_request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    run_request->run = false;

    if (run_client_ && rclcpp::ok()) {
        RCLCPP_INFO(g_node->get_logger(), "Stopping waypoint controller...");
        
        if (run_client_->wait_for_service(std::chrono::seconds(1))) {
            auto future = run_client_->async_send_request(run_request);
            rclcpp::spin_until_future_complete(g_node, future);
            RCLCPP_INFO(g_node->get_logger(), "Waypoint controller stopped");
        } else {
            RCLCPP_WARN(g_node->get_logger(), "Could not stop waypoint controller - service not available");
        }
    }
}

void ExecuteWaypoint::cleanup() {
    if (execution_started_) {
        stopWaypointController();
        execution_started_ = false;
        RCLCPP_INFO(g_node->get_logger(), "ExecuteWaypoint cleanup completed");
    }
}