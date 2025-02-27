#include "rov_mission_bt/behaviors/waypoint_behaviors.hpp"

// ClearWaypoints Implementation
ClearWaypoints::ClearWaypoints(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) 
{
    client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    RCLCPP_INFO(g_node->get_logger(), "Creating ClearWaypoints node");
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

BT::NodeStatus SetWaypoint::tick() {
    // v4 style port access
    auto x = getInput<double>("x");
    auto y = getInput<double>("y");
    auto z = getInput<double>("z");
    auto velocity = getInput<double>("velocity");

    if (!x || !y || !z || !velocity) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to get input parameters");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    request->x = x.value();
    request->y = y.value();
    request->z = z.value();
    request->desired_velocity = velocity.value();
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

    RCLCPP_INFO(g_node->get_logger(), "Successfully set waypoint: x=%.2f, y=%.2f, z=%.2f", 
                x.value(), y.value(), z.value());
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

BT::NodeStatus ExecuteWaypoint::onStart() {
    if (!first_run_) {
        return BT::NodeStatus::RUNNING;
    }

    if (!startWaypointController()) {
        return BT::NodeStatus::FAILURE;
    }

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

// BT::NodeStatus StationKeeping::onStart() {
//     auto duration = getInput<int>("duration");
//     auto heading = getInput<double>("heading");
    
//     if (!duration || !heading) {
//         RCLCPP_ERROR(g_node->get_logger(), "Failed to get duration or heading parameter for station keeping");
//         return BT::NodeStatus::FAILURE;
//     }

//     if (!startWaypointController(true)) {
//         return BT::NodeStatus::FAILURE;
//     }

//     if (!setCurrentPositionAsWaypoint(heading.value())) {
//         startWaypointController(false);
//         return BT::NodeStatus::FAILURE;
//     }

//     if (!startWaypointExecution()) {
//         startWaypointController(false);
//         return BT::NodeStatus::FAILURE;
//     }

//     start_time_ = std::chrono::steady_clock::now();
//     RCLCPP_INFO(g_node->get_logger(), "Starting station keeping for %d seconds with heading %.2f", 
//                 duration.value(), heading.value());
    
//     return BT::NodeStatus::RUNNING;
// }

// bool StationKeeping::setCurrentPositionAsWaypoint(double heading) {
//     auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    
//     // Set the current position as the waypoint
//     // Note: We'll use the same position that was last commanded
//     request->x = 0.0;  
//     request->y = 0.0;
//     request->z = 0.0;
//     request->desired_velocity = 0.0;
//     request->fixed_heading = true;
//     request->heading = heading;
    
//     if (!add_client_->wait_for_service(std::chrono::seconds(1))) {
//         RCLCPP_ERROR(g_node->get_logger(), "Add waypoint service not available");
//         return false;
//     }
    
//     auto future = add_client_->async_send_request(request);
//     if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_ERROR(g_node->get_logger(), "Failed to call add waypoint service");
//         return false;
//     }

//     auto result = future.get();
//     return result->accepted;
// }

// BT::NodeStatus StationKeeping::onRunning() {
//     auto duration = getInput<int>("duration");
//     if (!duration) {
//         RCLCPP_ERROR(g_node->get_logger(), "Failed to get duration parameter");
//         return BT::NodeStatus::FAILURE;
//     }

//     auto current_time = std::chrono::steady_clock::now();
//     auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
//         current_time - start_time_).count();

//     if (elapsed >= duration.value()) {
//         RCLCPP_INFO(g_node->get_logger(), "Station keeping completed");
//         return BT::NodeStatus::SUCCESS;
//     }

//     if (elapsed % 5 == 0) {
//         RCLCPP_INFO(g_node->get_logger(), "Station keeping: %ld seconds remaining", 
//                    duration.value() - elapsed);
//     }

//     return BT::NodeStatus::RUNNING;
// }

// void StationKeeping::onHalted() {
//     RCLCPP_INFO(g_node->get_logger(), "Station keeping halted");
//     startWaypointController(false);
// }

// bool StationKeeping::startWaypointController(bool run) {
//     auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
//     request->run = run;
    
//     if (!run_client_->wait_for_service(std::chrono::seconds(1))) {
//         RCLCPP_ERROR(g_node->get_logger(), "Run waypoint controller service not available");
//         return false;
//     }
    
//     auto future = run_client_->async_send_request(request);
//     if (rclcpp::spin_until_future_complete(g_node, future) != rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_ERROR(g_node->get_logger(), "Failed to call run waypoint controller service");
//         return false;
//     }

//     return true;
// }