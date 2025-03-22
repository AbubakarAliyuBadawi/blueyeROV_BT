// src/behaviors/station_keeping.cpp
#include "blueye_bt/behaviors/station_keeping.hpp"
#include <chrono>

using namespace std::chrono_literals;

StationKeeping::StationKeeping(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
{
    // Initialize service clients
    clear_client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    add_client_ = g_node->create_client<mundus_mir_msgs::srv::AddWaypoint>("/blueye/add_waypoint");
    run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>("/blueye/run_waypoint_controller");
    go_client_ = g_node->create_client<mundus_mir_msgs::srv::GoToWaypoints>("/blueye/go_to_waypoints");
    status_client_ = g_node->create_client<mundus_mir_msgs::srv::GetWaypointStatus>("/blueye/get_waypoint_status");
}

BT::NodeStatus StationKeeping::onStart() {
    // Get duration and heading parameters
    auto duration = getInput<int>("duration");
    auto heading = getInput<double>("heading");
    
    // Get optional altitude mode parameters
    bool altitude_mode = false;
    double target_altitude = 2.0;
    getInput("altitude_mode", altitude_mode);
    getInput("target_altitude", target_altitude);
    
    if (!duration || !heading) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to get duration or heading parameter for station keeping");
        return BT::NodeStatus::FAILURE;
    }
    
    // Clear any existing waypoints
    if (!clearWaypoints()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to clear waypoints");
        return BT::NodeStatus::FAILURE;
    }
    
    // Start waypoint controller
    if (!startWaypointController(true)) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start waypoint controller");
        return BT::NodeStatus::FAILURE;
    }
    
    // Add waypoint with the specified position, heading, and altitude mode if enabled
    if (!addWaypoint(heading.value(), altitude_mode, target_altitude)) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to add waypoint");
        startWaypointController(false);
        return BT::NodeStatus::FAILURE;
    }
    
    // Start executing (tells the controller to maintain position)
    if (!startWaypointExecution()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start waypoint execution");
        startWaypointController(false);
        return BT::NodeStatus::FAILURE;
    }
    
    // Record start time for duration tracking
    start_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(g_node->get_logger(), "Starting station keeping for %d seconds with heading %.2f°", 
                duration.value(), heading.value());
    
    if (altitude_mode) {
        RCLCPP_INFO(g_node->get_logger(), "Using altitude mode with target altitude %.2f meters", target_altitude);
    }
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StationKeeping::onRunning() {
    auto duration = getInput<int>("duration");
    if (!duration) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to get duration parameter");
        return BT::NodeStatus::FAILURE;
    }
    
    // Check if the duration has elapsed
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - start_time_).count();
    
    if (elapsed >= duration.value()) {
        RCLCPP_INFO(g_node->get_logger(), "Station keeping completed after %d seconds", 
                   static_cast<int>(elapsed));
        
        // Clean up resources before returning success
        stopExecution();
        
        return BT::NodeStatus::SUCCESS;
    }
    
    // Log periodic updates (every 5 seconds)
    if (elapsed % 5 == 0) {
        static int last_log_time = -1;
        if (elapsed != last_log_time) {
            RCLCPP_INFO(g_node->get_logger(), "Station keeping: %ld seconds remaining", 
                       duration.value() - elapsed);
            last_log_time = elapsed;
        }
    }
    
    return BT::NodeStatus::RUNNING;
}

bool StationKeeping::clearWaypoints() {
    RCLCPP_INFO(g_node->get_logger(), "Clearing waypoints...");
    if (!clear_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Clear waypoints service not available");
        return false;
    }
    
    auto request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    auto future = clear_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call clear waypoints service");
        return false;
    }
    
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Clear waypoints request was not accepted");
        return false;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Waypoints cleared successfully");
    return true;
}

bool StationKeeping::addWaypoint(double heading, bool altitude_mode, double target_altitude) {
    // Try to get position parameters if provided
    auto x_opt = getInput<double>("x");
    auto y_opt = getInput<double>("y");
    auto z_opt = getInput<double>("z");
    
    // Check if all position parameters are provided
    bool has_position = x_opt.has_value() && y_opt.has_value() && z_opt.has_value();
    
    if (!add_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Add waypoint service not available");
        return false;
    }
    
    auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    
    if (has_position) {
        // Use provided position
        request->x = x_opt.value();
        request->y = y_opt.value();
        request->z = z_opt.value();
        
        if (altitude_mode) {
            RCLCPP_INFO(g_node->get_logger(), 
                      "Setting station keeping at specified position (x: %.2f, y: %.2f, z: %.2f, heading: %.2f°) with altitude mode, target altitude: %.2f m",
                      request->x, request->y, request->z, heading, target_altitude);
        } else {
            RCLCPP_INFO(g_node->get_logger(), 
                      "Setting station keeping at specified position (x: %.2f, y: %.2f, z: %.2f, heading: %.2f°)",
                      request->x, request->y, request->z, heading);
        }
    } else {
        // Get status to see current position (for logging)
        auto status_request = std::make_shared<mundus_mir_msgs::srv::GetWaypointStatus::Request>();
        auto status_future = status_client_->async_send_request(status_request);
        
        if (rclcpp::spin_until_future_complete(g_node, status_future, 2s) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = status_future.get();
            RCLCPP_INFO(g_node->get_logger(), "Current status: %s", result->status_code.c_str());
        }
        
        // Use 0,0,0 which the controller interprets as "current position"
        request->x = 0.0;
        request->y = 0.0;
        request->z = 0.0;
        
        if (altitude_mode) {
            RCLCPP_INFO(g_node->get_logger(), 
                      "Setting station keeping at current position with heading: %.2f° and altitude mode, target altitude: %.2f m", 
                      heading, target_altitude);
        } else {
            RCLCPP_INFO(g_node->get_logger(), 
                      "Setting station keeping at current position with heading: %.2f°", 
                      heading);
        }
    }
    
    request->desired_velocity = 0.2;  // Low velocity for station keeping
    request->fixed_heading = true;    // Critical: this makes the controller respect our heading
    request->heading = heading * M_PI / 180.0;  // Convert degrees to radians
    
    // Add altitude mode if enabled
    request->altitude_mode = altitude_mode;
    request->target_altitude = target_altitude;
    
    auto future = add_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call add waypoint service");
        return false;
    }
    
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Add waypoint request was not accepted");
        return false;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Waypoint added successfully");
    return true;
}

bool StationKeeping::startWaypointController(bool run) {
    RCLCPP_INFO(g_node->get_logger(), run ? "Starting waypoint controller..." : "Stopping waypoint controller...");
    
    if (!run_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Run waypoint controller service not available");
        return false;
    }
    
    auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    request->run = run;
    
    auto future = run_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call run waypoint controller service");
        return false;
    }
    
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), 
                    run ? "Start waypoint controller request was not accepted" : 
                         "Stop waypoint controller request was not accepted");
        return false;
    }
    
    RCLCPP_INFO(g_node->get_logger(), 
               run ? "Waypoint controller started successfully" : 
                    "Waypoint controller stopped successfully");
    return true;
}

bool StationKeeping::startWaypointExecution() {
    RCLCPP_INFO(g_node->get_logger(), "Starting station keeping execution...");
    
    if (!go_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Go to waypoints service not available");
        return false;
    }
    
    auto request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
    request->run = true;
    
    auto future = go_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call go to waypoints service");
        return false;
    }
    
    auto result = future.get();
    if (!result->accepted) {
        RCLCPP_ERROR(g_node->get_logger(), "Start waypoint execution request was not accepted");
        return false;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Station keeping execution started successfully");
    return true;
}

void StationKeeping::stopExecution() {
    // First stop the waypoint execution
    if (go_client_->wait_for_service(5s)) {
        auto go_request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
        go_request->run = false;
        auto future = go_client_->async_send_request(go_request);
        rclcpp::spin_until_future_complete(g_node, future, 1s);
    }
    
    // Then stop the controller
    startWaypointController(false);
    
    // Finally clear any waypoints
    clearWaypoints();
    
    RCLCPP_INFO(g_node->get_logger(), "Station keeping execution stopped");
}

void StationKeeping::onHalted() {
    RCLCPP_INFO(g_node->get_logger(), "Station keeping halted");
    stopExecution();
}