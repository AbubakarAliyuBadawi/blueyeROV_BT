// safe_navigate_to_waypoint.cpp
#include "blueye_bt/behaviors/safe_navigate_to_waypoint.hpp"
#include <chrono>
#include <cstring>
#include <cmath>

using namespace std::chrono_literals;

SafeNavigateToWaypoint::SafeNavigateToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : NavigateToWaypoint(name, config) {
    
    // Get parameters from ports
    getInput("safety_distance", safety_distance_);
    getInput("deviation_distance", deviation_distance_);
    getInput("deviation_velocity", deviation_velocity_);
    getInput("obstacle_avoidance_enabled", obstacle_avoidance_enabled_);
    getInput("waypoint_resume_delay", waypoint_resume_delay_);
    
    // Create our own service clients
    sa_clear_client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    sa_add_client_ = g_node->create_client<mundus_mir_msgs::srv::AddWaypoint>("/blueye/add_waypoint");
    sa_go_client_ = g_node->create_client<mundus_mir_msgs::srv::GoToWaypoints>("/blueye/go_to_waypoints");
    
    // Create sonar subscription
    sonar_sub_ = g_node->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        "/mundus_mir/sonar", 10, 
        std::bind(&SafeNavigateToWaypoint::sonarCallback, this, std::placeholders::_1));
    
    // Create odometry subscription
    odom_sub_ = g_node->create_subscription<nav_msgs::msg::Odometry>(
        "/blueye/odometry_frd/gt", 10,
        std::bind(&SafeNavigateToWaypoint::odomCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(g_node->get_logger(), "SafeNavigateToWaypoint initialized with safety_distance=%.2f, deviation_distance=%.2f",
               safety_distance_, deviation_distance_);
}

BT::NodeStatus SafeNavigateToWaypoint::onStart() {
    RCLCPP_INFO(g_node->get_logger(), "Starting SafeNavigateToWaypoint...");
    
    // Reset avoidance state
    obstacle_detected_ = false;
    is_avoiding_ = false;
    is_recovering_ = false;
    
    // Store original waypoint
    getInput("x", original_waypoint_.x);
    getInput("y", original_waypoint_.y);
    getInput("z", original_waypoint_.z);
    getInput("velocity", original_waypoint_.velocity);
    getInput("fixed_heading", original_waypoint_.fixed_heading);
    getInput("heading", original_waypoint_.heading);
    
    have_original_waypoint_ = true;
    
    RCLCPP_INFO(g_node->get_logger(), "Original waypoint: x=%.2f, y=%.2f, z=%.2f, velocity=%.2f",
               original_waypoint_.x, original_waypoint_.y, original_waypoint_.z, original_waypoint_.velocity);
    
    // Start normal navigation
    return NavigateToWaypoint::onStart();
}

BT::NodeStatus SafeNavigateToWaypoint::onRunning() {
    // Check if we're already avoiding or need to start
    if (obstacle_avoidance_enabled_ && !is_avoiding_ && !is_recovering_) {
        if (checkForObstacles()) {
            RCLCPP_INFO(g_node->get_logger(), "Obstacle detected while navigating! Starting avoidance maneuver");
            startObstacleAvoidance();
            // Let the base class handle navigation after we've started avoidance
            return BT::NodeStatus::RUNNING;
        }
    }
    
    // Check avoidance status and handle transitions
    if (is_avoiding_ || is_recovering_) {
        checkAvoidanceStatus();
    }
    
    // Continue normal navigation
    return NavigateToWaypoint::onRunning();
}

void SafeNavigateToWaypoint::onHalted() {
    RCLCPP_INFO(g_node->get_logger(), "SafeNavigateToWaypoint halted");
    is_avoiding_ = false;
    is_recovering_ = false;
    NavigateToWaypoint::onHalted();
}

bool SafeNavigateToWaypoint::sa_clearWaypoints() {
    // Implementation of our own clearWaypoints method
    RCLCPP_INFO(g_node->get_logger(), "Clearing waypoints...");
    auto request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    auto future = sa_clear_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to clear waypoints");
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

bool SafeNavigateToWaypoint::sa_addWaypoint(double x, double y, double z, double velocity, 
                                         bool fixed_heading, double heading, 
                                         bool altitude_mode, double target_altitude) {
    // Implementation of our own addWaypoint method
    RCLCPP_INFO(g_node->get_logger(), "Adding waypoint...");
    auto request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    request->x = x;
    request->y = y;
    request->z = z;
    request->desired_velocity = velocity;
    request->fixed_heading = fixed_heading;
    request->heading = heading;
    
    auto future = sa_add_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to add waypoint");
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

bool SafeNavigateToWaypoint::sa_startWaypointExecution() {
    // Implementation of our own startWaypointExecution method
    RCLCPP_INFO(g_node->get_logger(), "Starting waypoint execution...");
    auto request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
    request->run = true;
    auto future = sa_go_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start waypoint execution");
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

void SafeNavigateToWaypoint::sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
    if (g_node->now() < avoidance_cooldown_end_) {
        return;
    }
    if (!obstacle_avoidance_enabled_ || !have_pose_ || is_avoiding_ || is_recovering_) {
        return;
    }
    
    // Divide the sonar beams into sectors (left, center, right)
    int total_beams = msg->beam_directions.size();
    int left_sector_start = 0;
    int left_sector_end = total_beams / 3;
    int center_sector_start = left_sector_end;
    int center_sector_end = 2 * total_beams / 3;
    int right_sector_start = center_sector_end;
    int right_sector_end = total_beams;
    
    // Calculate minimum distances for each sector
    min_dist_left_ = findMinDistanceInSector(msg, left_sector_start, left_sector_end);
    min_dist_center_ = findMinDistanceInSector(msg, center_sector_start, center_sector_end);
    min_dist_right_ = findMinDistanceInSector(msg, right_sector_start, right_sector_end);
    
    RCLCPP_DEBUG(g_node->get_logger(), "Min distances - Left: %.2f, Center: %.2f, Right: %.2f",
               min_dist_left_, min_dist_center_, min_dist_right_);
}

void SafeNavigateToWaypoint::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    have_pose_ = true;
    
    // Calculate yaw from quaternion without using tf2
    // Yaw from quaternion: atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto& q = current_pose_.orientation;
    current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

float SafeNavigateToWaypoint::findMinDistanceInSector(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg,
                                                    int sector_start, int sector_end) {
    float min_distance = std::numeric_limits<float>::max();
    float intensity_threshold = 0.2f;  // Threshold for obstacle detection
    
    int obstacles_detected = 0;
    
    // For each beam in this sector
    for (int beam_idx = sector_start; beam_idx < sector_end; beam_idx++) {
        // For each range in this beam
        for (size_t range_idx = 0; range_idx < msg->ranges.size(); range_idx++) {
            // Calculate the index in the flattened data array
            size_t data_idx = beam_idx * msg->ranges.size() + range_idx;
            if (data_idx * 4 + 3 >= msg->image.data.size()) continue;  // Safety check
            
            // Convert 4 bytes to float (for DTYPE_FLOAT32)
            float intensity = 0.0f;
            if (msg->image.dtype == 8) {  // DTYPE_FLOAT32
                uint8_t bytes[4] = {
                    msg->image.data[data_idx * 4],
                    msg->image.data[data_idx * 4 + 1],
                    msg->image.data[data_idx * 4 + 2],
                    msg->image.data[data_idx * 4 + 3]
                };
                memcpy(&intensity, bytes, sizeof(float));
            }
            
            // If intensity exceeds threshold, consider it an obstacle
            if (intensity > intensity_threshold) {
                float range = msg->ranges[range_idx];
                obstacles_detected++;
                
                if (range < min_distance) {
                    min_distance = range;
                    RCLCPP_DEBUG(g_node->get_logger(), "New minimum distance: %.2f at beam %d, range %zu",
                               min_distance, beam_idx, range_idx);
                }
                break;  // Found closest obstacle in this beam
            }
        }
    }
    
    if (min_distance == std::numeric_limits<float>::max()) {
        min_distance = msg->ranges.back();  // Return max range if no obstacle detected
    }
    
    return min_distance;
}

bool SafeNavigateToWaypoint::checkForObstacles() {
    // Check if center sector has obstacle within safety distance

    if (g_node->now() < avoidance_cooldown_end_) {
        return false;
    }
    if (min_dist_center_ < safety_distance_) {
        RCLCPP_INFO(g_node->get_logger(), "Obstacle detected! Distance (%.2f) < Safety threshold (%.2f)",
                    min_dist_center_, safety_distance_);
        return true;
    }
    return false;
}

void SafeNavigateToWaypoint::startObstacleAvoidance() {
    RCLCPP_INFO(g_node->get_logger(), "==== OBSTACLE AVOIDANCE STARTING ====");
    RCLCPP_INFO(g_node->get_logger(), "Obstacle detected at %.2f meters. Starting avoidance maneuver.", min_dist_center_);
    
    is_avoiding_ = true;
    avoidance_start_time_ = g_node->now();
    
    // Create deviation waypoint
    // Determine if we should go left or right - go towards the side with more space
    // bool go_left = min_dist_left_ > min_dist_right_;
    bool go_left = min_dist_left_ < min_dist_right_;
    
    // Calculate 25-degree deviation angle
    // float deviation_angle = current_yaw_ + (go_left ? (25.0 * M_PI/180.0) : -(25.0 * M_PI/180.0));

    // Calculate 40-degree deviation angle instead of 25-degree
    float deviation_angle = current_yaw_ + (go_left ? (40.0 * M_PI/180.0) : -(40.0 * M_PI/180.0));
    
    // Calculate deviation point
    avoidance_waypoint_.x = current_pose_.position.x + deviation_distance_ * cos(deviation_angle);
    avoidance_waypoint_.y = current_pose_.position.y + deviation_distance_ * sin(deviation_angle);
    avoidance_waypoint_.z = original_waypoint_.z;  // Keep original z
    avoidance_waypoint_.velocity = deviation_velocity_;
    avoidance_waypoint_.fixed_heading = false;
    avoidance_waypoint_.heading = deviation_angle;
    
    RCLCPP_INFO(g_node->get_logger(), "Creating avoidance waypoint at x=%.2f, y=%.2f (going %s)",
               avoidance_waypoint_.x, avoidance_waypoint_.y, go_left ? "left" : "right");
    
    // Clear existing waypoints using our own method
    if (!sa_clearWaypoints()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to clear waypoints for avoidance");
        is_avoiding_ = false;
        return;
    }
    
    // Create new avoidance waypoint
    if (!sa_addWaypoint(avoidance_waypoint_.x, avoidance_waypoint_.y, avoidance_waypoint_.z,
                      avoidance_waypoint_.velocity, avoidance_waypoint_.fixed_heading, 
                      avoidance_waypoint_.heading, false, 0.0)) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to add avoidance waypoint");
        is_avoiding_ = false;
        return;
    }
    
    // Start waypoint execution
    if (!sa_startWaypointExecution()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start avoidance waypoint execution");
        is_avoiding_ = false;
        return;
    }
}

void SafeNavigateToWaypoint::startRecovery() {
    is_avoiding_ = false;
    is_recovering_ = true;
    recovery_start_time_ = g_node->now();
    
    RCLCPP_INFO(g_node->get_logger(), "Avoidance waypoint reached, starting recovery phase");
}

void SafeNavigateToWaypoint::returnToOriginalWaypoint() {
    RCLCPP_INFO(g_node->get_logger(), "Recovery complete, returning to original waypoint");
    
    is_recovering_ = false;
    avoidance_cooldown_end_ = g_node->now() + rclcpp::Duration::from_seconds(15.0); 
    
    // Reset distance values to ensure we're not using stale data
    min_dist_left_ = std::numeric_limits<float>::max();
    min_dist_center_ = std::numeric_limits<float>::max();
    min_dist_right_ = std::numeric_limits<float>::max();

    // Clear existing waypoints
    if (!sa_clearWaypoints()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to clear waypoints for return");
        return;
    }
    
    // Add original waypoint back
    if (!sa_addWaypoint(original_waypoint_.x, original_waypoint_.y, original_waypoint_.z,
                      original_waypoint_.velocity, original_waypoint_.fixed_heading, 
                      original_waypoint_.heading, false, 0.0)) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to add original waypoint");
        return;
    }
    
    // Start execution
    if (!sa_startWaypointExecution()) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to start original waypoint execution");
        return;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Successfully returning to original waypoint");
}

void SafeNavigateToWaypoint::checkAvoidanceStatus() {
    // If we're avoiding, check if we've reached the avoidance waypoint
    if (is_avoiding_) {
        float dist_to_waypoint = std::sqrt(
            std::pow(current_pose_.position.x - avoidance_waypoint_.x, 2) +
            std::pow(current_pose_.position.y - avoidance_waypoint_.y, 2));
        
        // If we've reached the avoidance waypoint or timeout, start recovery
        if (dist_to_waypoint < 1.0 || 
            (g_node->now() - avoidance_start_time_).seconds() > 15.0) {
            startRecovery();
        }
    }
    
    // If we're recovering, check if it's time to return to original waypoint
    if (is_recovering_) {
        if ((g_node->now() - recovery_start_time_).seconds() > waypoint_resume_delay_) {
            returnToOriginalWaypoint();
        }
    }
}