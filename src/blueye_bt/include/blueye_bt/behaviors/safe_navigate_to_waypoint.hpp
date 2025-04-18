// safe_navigate_to_waypoint.hpp
#pragma once
#include "blueye_bt/behaviors/navigate_to_waypoint.hpp"
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <cstring>
#include <limits>

class SafeNavigateToWaypoint : public NavigateToWaypoint {
public:
    SafeNavigateToWaypoint(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts() {
        // Add obstacle avoidance parameters to the base class ports
        auto base_ports = NavigateToWaypoint::providedPorts();
        base_ports.insert(BT::InputPort<double>("safety_distance", 5.0, "Minimum distance to obstacles"));
        base_ports.insert(BT::InputPort<double>("deviation_distance", 12.0, "Distance to deviation point"));
        base_ports.insert(BT::InputPort<double>("deviation_velocity", 0.2, "Velocity during avoidance"));
        base_ports.insert(BT::InputPort<bool>("obstacle_avoidance_enabled", true, "Enable obstacle avoidance"));
        base_ports.insert(BT::InputPort<double>("waypoint_resume_delay", 0.1, "Delay before returning to original path"));
        return base_ports;
    }
    
protected:
    // Override NavigateToWaypoint methods
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    
private:
    // Service clients (needed since we can't access base class's private clients)
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr sa_clear_client_;
    rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr sa_add_client_;
    rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr sa_go_client_;
    
    // Subscriptions
    rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Time avoidance_cooldown_end_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    
    // State variables
    bool obstacle_detected_ = false;
    bool is_avoiding_ = false;
    bool is_recovering_ = false;
    bool have_pose_ = false;
    bool have_original_waypoint_ = false;
    float min_dist_left_ = std::numeric_limits<float>::max();
    float min_dist_center_ = std::numeric_limits<float>::max();
    float min_dist_right_ = std::numeric_limits<float>::max();
    float current_yaw_ = 0.0f;
    geometry_msgs::msg::Pose current_pose_;
    rclcpp::Time avoidance_start_time_;
    rclcpp::Time recovery_start_time_;
    
    
    // Original and avoidance waypoint storage
    struct Waypoint {
        double x;
        double y;
        double z;
        double velocity;
        bool fixed_heading;
        double heading;
    };
    Waypoint original_waypoint_;
    Waypoint avoidance_waypoint_;
    std::mutex waypoint_mutex_;
    
    // Parameters
    double safety_distance_ = 5.0;
    double deviation_distance_ = 10.0;
    double deviation_velocity_ = 0.3;
    double waypoint_resume_delay_ = 0.1;
    bool obstacle_avoidance_enabled_ = true;
    
    // Helper methods that replicate the base class functionality
    bool sa_clearWaypoints();
    bool sa_addWaypoint(double x, double y, double z, double velocity, bool fixed_heading, 
                      double heading, bool altitude_mode, double target_altitude);
    bool sa_startWaypointExecution();
    
    // Helper methods for obstacle avoidance
    void sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    float findMinDistanceInSector(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg,
                                 int sector_start, int sector_end);
    bool checkForObstacles();
    void startObstacleAvoidance();
    void startRecovery();
    void returnToOriginalWaypoint();
    void checkAvoidanceStatus();
};