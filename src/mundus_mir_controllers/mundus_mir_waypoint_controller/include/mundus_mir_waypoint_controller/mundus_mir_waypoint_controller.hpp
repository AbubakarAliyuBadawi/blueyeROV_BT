#ifndef MUNDUS_MIR_WAYPOINT_CONTROLLER_H_
#define MUNDUS_MIR_WAYPOINT_CONTROLLER_H_

// C++ includes
#include <vector>
#include <string>
#include <sstream>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// Waypoint services
#include "mundus_mir_msgs/srv/add_waypoint.hpp"
#include "mundus_mir_msgs/srv/insert_waypoint.hpp"
#include "mundus_mir_msgs/srv/remove_waypoint.hpp"
#include "mundus_mir_msgs/srv/get_waypoints.hpp"
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"
#include "mundus_mir_msgs/srv/go_to_waypoints.hpp"
#include "mundus_mir_msgs/srv/get_waypoint_status.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

using namespace std;

// Datastructure used to store information about waypoints
struct Waypoint
{
    float x;
    float y;
    float z;
    float desired_speed;
    bool fixed_heading;
    float heading;

    // Consturctor to give a default desired speed value of 0.5m/s
    Waypoint(float x = 0.0, float y = 0.0, float z = 0.0, float desired_speed = 0.5, bool fixed_heading = false, float heading = 0.0)
    : x(x), y(y), z(z), desired_speed(desired_speed), fixed_heading(fixed_heading), heading(heading) {}
};

class WaypointController : public rclcpp::Node
{
    /*
    Summary:  
    
    
    
    
    
    */

    // Methods
    public:
        WaypointController();
        ~WaypointController();

    //  Methods
    private:

        // administration
        void get_ros_parameters();
        void setup_interfaces();
        string waypoints_to_string();

        // Controller callback is run everytime a new odometry message is received
        void controller_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Station Keep - Controller function to keep the robot on the stationary waypoint
        void station_keep(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Go to waypoint - Controller function to make the robot go to the first waypoint in waypoints_
        void go_to_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Waypoint service callbacks
        void add_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Request> request,
                          std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Response> response);
        void insert_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Request> request,
                             std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Response> response);
        void remove_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Request> request,
                            std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Response> response);
        void get_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Request> request,
                          std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Response> response);
        void clear_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Request> request,
                            std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Response> response);
        void go_to_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Request> request,
                          std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Response> response);
        void get_waypoint_status(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Request> request,
                          std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Response> response);
        void run_waypoint_controller(const std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Request> request,    
                          std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Response> response);
        
    // Members
    private:

        // Controller variables
        bool run_controller_ = false;
        bool go_to_waypoint_ = false;
        bool stationary_waypoint_set_ = false;
        vector<Waypoint> waypoints_;
        Waypoint stationary_waypoint_;

        // ROS Parameters
        string odometry_topic_, desired_velocity_topic_;
        string add_waypoint_srv_name_, insert_waypoint_srv_name_, remove_waypoint_srv_name_;
        string get_waypoints_srv_name_, clear_waypoints_srv_name_, go_to_waypoints_srv_name_;
        string get_waypoint_status_srv_name_, run_waypoint_controller_srv_name_;
        float circle_of_acceptance_, heading_tolerance_, heading_speed_, heave_speed_, station_keep_speed_;

        // Current odometry subscriber, used to get the robots current position. 
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

        // Desired velocity publisher 
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr desired_velocity_pub_;

        // Waypoint services
        rclcpp::Service<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::InsertWaypoint>::SharedPtr insert_waypoint_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::RemoveWaypoint>::SharedPtr remove_waypoint_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::GetWaypoints>::SharedPtr get_waypoints_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_waypoints_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr go_to_waypoints_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr get_waypoint_status_srv_;
        rclcpp::Service<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_waypoint_controller_srv_;   

};

#endif