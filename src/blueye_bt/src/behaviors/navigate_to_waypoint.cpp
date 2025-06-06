#include "blueye_bt/behaviors/navigate_to_waypoint.hpp"
#include <chrono>

using namespace std::chrono_literals;

bool NavigateToWaypoint::enableController() {
    if (controller_enabled_) {
        return true;
    }

    RCLCPP_INFO(g_node->get_logger(), "Enabling waypoint controller...");
    
    if (!run_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Run controller service not available after 2s");
        return false;
    }

    auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    request->run = true;
    auto future = run_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 10s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to enable waypoint controller");
        return false;
    }
    
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Enable controller request was not accepted");
        return false;
    }

    controller_enabled_ = true;
    RCLCPP_INFO(g_node->get_logger(), "Waypoint controller enabled successfully");
    return true;
}

BT::NodeStatus NavigateToWaypoint::onStart() {
    RCLCPP_INFO(g_node->get_logger(), "Starting NavigateToWaypoint node...");

    // First ensure the controller is enabled
    if (!enableController()) {
        return BT::NodeStatus::FAILURE;
    }

    // Get required parameters from ports
    double x, y, z, velocity, heading = 0.0;
    bool fixed_heading = false, altitude_mode = false;
    double target_altitude = 2.0;
    
    if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z)) {
        RCLCPP_ERROR(g_node->get_logger(), "Missing required waypoint coordinates");
        return BT::NodeStatus::FAILURE;
    }
    
    // Get optional parameters with defaults
    getInput("velocity", velocity);
    getInput("fixed_heading", fixed_heading);
    getInput("heading", heading);
    getInput("altitude_mode", altitude_mode);
    getInput("target_altitude", target_altitude);
    
    RCLCPP_INFO(g_node->get_logger(), "Received waypoint: x=%.2f, y=%.2f, z=%.2f, v=%.2f, fixed_heading=%s, heading=%.2f",
               x, y, z, velocity, fixed_heading ? "true" : "false", heading);
    
    if (altitude_mode) {
        RCLCPP_INFO(g_node->get_logger(), "Note: Altitude mode is not supported in this version. Using fixed z value.");
    }

    // Check service availability first
    if (!clear_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Clear waypoints service not available after 2s");
        return BT::NodeStatus::FAILURE;
    }
    if (!add_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Add waypoint service not available after 2s");
        return BT::NodeStatus::FAILURE;
    }
    if (!go_client_->wait_for_service(10s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Run waypoint controller service not available after 2s");
        return BT::NodeStatus::FAILURE;
    }

    if (!clearWaypoints()) {
        return BT::NodeStatus::FAILURE;
    }

    if (!addWaypoint(x, y, z, velocity, fixed_heading, heading, altitude_mode, target_altitude)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!startWaypointController()) {
        return BT::NodeStatus::FAILURE;
    }

    first_run_ = false;
    execution_started_ = false;
    RCLCPP_INFO(g_node->get_logger(), "NavigateToWaypoint started successfully");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning() {
    if (!execution_started_) {
        if (!startWaypointExecution()) {
            return BT::NodeStatus::FAILURE;
        }
        execution_started_ = true;
        RCLCPP_INFO(g_node->get_logger(), "Waypoint execution started");
        return BT::NodeStatus::RUNNING;
    }

    // Check waypoint status
    auto request = std::make_shared<mundus_mir_msgs::srv::GetWaypointStatus::Request>();
    auto future = status_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to get waypoint status");
        return BT::NodeStatus::FAILURE;
    }

    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Waypoint status request not accepted");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(g_node->get_logger(), "Status: %s", result->status_code.c_str());

    // If no waypoints left, we've reached our destination
    if (result->status_code.find("Amount of waypoints: 0") != std::string::npos) {
        RCLCPP_INFO(g_node->get_logger(), "Waypoint reached successfully");
        return BT::NodeStatus::SUCCESS;
    }
    
    // If we're in station-keeping mode, we've also reached our destination
    if (result->status_code.find("Station Keep On Position") != std::string::npos) {
        RCLCPP_INFO(g_node->get_logger(), "Waypoint reached, now in station-keeping mode");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void NavigateToWaypoint::onHalted() {
    RCLCPP_INFO(g_node->get_logger(), "NavigateToWaypoint halted");
    cleanup();
}

bool NavigateToWaypoint::clearWaypoints() {
    RCLCPP_INFO(g_node->get_logger(), "Clearing waypoints...");
    auto request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    auto future = clear_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to clear waypoints - service call timeout");
        return false;
    }
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Clear waypoints request was not accepted by the service");
        return false;
    }
    RCLCPP_INFO(g_node->get_logger(), "Waypoints cleared successfully");
    return true;
}

bool NavigateToWaypoint::addWaypoint(double x, double y, double z, double velocity, 
                                     bool fixed_heading, double heading,
                                     bool altitude_mode, double target_altitude) {
    RCLCPP_INFO(g_node->get_logger(), "Adding waypoint...");
    auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    request->x = x;
    request->y = y;
    request->z = z;
    request->desired_velocity = velocity;
    request->fixed_heading = fixed_heading;
    request->heading = heading;
    
    // Note: Not setting altitude mode parameters as they don't exist in this version
    // Just log what would have been done
    if (altitude_mode) {
        RCLCPP_INFO(g_node->get_logger(), "Note: Altitude mode requested but not supported in this version. Using fixed z=%.2f instead.", z);
    }

    RCLCPP_INFO(g_node->get_logger(), "Sending waypoint: x=%.2f, y=%.2f, z=%.2f, v=%.2f, fixed_heading=%s, heading=%.2f",
                x, y, z, velocity, fixed_heading ? "true" : "false", heading);

    auto future = add_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to add waypoint - service call timeout");
        return false;
    }
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Add waypoint request was not accepted by the service");
        return false;
    }
    RCLCPP_INFO(g_node->get_logger(), "Waypoint added successfully");
    return true;
}

bool NavigateToWaypoint::startWaypointController() {
    RCLCPP_INFO(g_node->get_logger(), "Starting waypoint controller...");
    auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    request->run = true;
    auto future = run_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start waypoint controller - service call timeout");
        return false;
    }
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Start waypoint controller request was not accepted");
        return false;
    }
    RCLCPP_INFO(g_node->get_logger(), "Waypoint controller started successfully");
    return true;
}

bool NavigateToWaypoint::startWaypointExecution() {
    RCLCPP_INFO(g_node->get_logger(), "Starting waypoint execution...");
    auto request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
    request->run = true;
    auto future = go_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start waypoint execution - service call timeout");
        return false;
    }
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Start waypoint execution request was not accepted");
        return false;
    }
    RCLCPP_INFO(g_node->get_logger(), "Waypoint execution started successfully");
    return true;
}

void NavigateToWaypoint::stopWaypointController() {
    RCLCPP_INFO(g_node->get_logger(), "Stopping waypoint controller...");
    auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    request->run = false;
    auto future = run_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->accepted) {
            RCLCPP_WARN(g_node->get_logger(), "Stop waypoint controller request was not accepted");
        } else {
            RCLCPP_INFO(g_node->get_logger(), "Waypoint controller stopped successfully");
            controller_enabled_ = false;
        }
    } else {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to stop waypoint controller - service call timeout");
    }
}

void NavigateToWaypoint::cleanup() {
    if (!first_run_) {
        RCLCPP_INFO(g_node->get_logger(), "Cleaning up NavigateToWaypoint...");
        stopWaypointController();
        clearWaypoints();
    }
}