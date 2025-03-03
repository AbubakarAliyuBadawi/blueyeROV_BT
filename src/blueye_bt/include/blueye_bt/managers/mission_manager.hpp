#ifndef MISSION_MANAGER_HPP
#define MISSION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <map>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "mundus_mir_msgs/srv/mission_manager.hpp"
#include "mundus_mir_msgs/srv/go_to_waypoints.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"
#include <nav_msgs/msg/odometry.hpp>

class MissionManager : public rclcpp::Node {
public:
    // Constructor and destructor
    MissionManager();
    ~MissionManager();

private:
    // Mission files
    std::string pipeline_mission_file_;
    std::string lawnmower_mission_file_;
    int station_keeping_duration_;
    
    // Current mission state
    std::string current_mission_;
    std::shared_ptr<BT::Tree> active_tree_;
    std::thread* mission_thread_;
    std::mutex mission_mutex_;
    std::condition_variable mission_cv_;
    bool mission_running_;
    bool mission_should_stop_;
    
    // Last known position for safe state return
    double current_x_;
    double current_y_;
    double current_z_;
    double current_heading_;
    std::mutex position_mutex_;
    
    // ROS interfaces
    rclcpp::Service<mundus_mir_msgs::srv::MissionManager>::SharedPtr mission_service_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr waypoint_controller_client_;
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_waypoints_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // BT factory
    BT::BehaviorTreeFactory factory_;
    
    // Private methods
    void init_bt_factory();
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    void handle_mission_switch(
        const std::shared_ptr<mundus_mir_msgs::srv::MissionManager::Request> request,
        std::shared_ptr<mundus_mir_msgs::srv::MissionManager::Response> response);
    
    bool transition_to_safe_state();
    
    void stop_current_mission();
    
    bool start_pipeline_mission();
    
    bool start_lawnmower_mission();
    
    bool start_mission(const std::string& mission_file);
};

#endif // MISSION_MANAGER_HPP