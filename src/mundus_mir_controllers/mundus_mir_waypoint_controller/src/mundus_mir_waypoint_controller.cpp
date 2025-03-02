#include "mundus_mir_waypoint_controller/mundus_mir_waypoint_controller.hpp"

float calculate_distance(const Waypoint &waypoint, const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Calculate distance between waypoint and current position
    return sqrt(pow(waypoint.x - msg->pose.pose.position.x, 2) + pow(waypoint.y - msg->pose.pose.position.y, 2));
}

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
{
    // Convert quaternion to yaw (rotation around Z axis)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

WaypointController::WaypointController() : Node("mundus_mir_waypoint_controller")
{
    // Fetch ROS parameters
    get_ros_parameters();

    // Set up publishers, subscribers and services
    setup_interfaces();
}

WaypointController::~WaypointController()
{
}

void WaypointController::controller_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check if we should run the controller
    if (!run_controller_)
    {
        return;
    }

    // Check if waypoint following should be run
    if (go_to_waypoint_)
    {

        // Check current waypoint against odometry
        if (calculate_distance(waypoints_.front(), msg) < circle_of_acceptance_)
        {
            // If this is the last waypoint we should go to stationkeep
            if (waypoints_.size() == 1)
            {
                stationary_waypoint_ = waypoints_.front();
                // If the current waypoint does not have a fixed heading, set it to the current heading
                if (!stationary_waypoint_.fixed_heading) {
                    stationary_waypoint_.heading = get_yaw_from_quaternion(msg->pose.pose.orientation); 
                    stationary_waypoint_.fixed_heading = true;
                }
                go_to_waypoint_ = false;
                stationary_waypoint_set_ = true;
            }
            // If we are close to the waypoint, remove it
            waypoints_.erase(waypoints_.begin());
            return;
        }

        // Go to the current waypoint
        go_to_waypoint(msg);
        return;
    }

    // If not station keep on stationary_waypoint
    if (stationary_waypoint_set_)
    {
        station_keep(msg);
        return;
    }
    // If it is not set, it should be set to current pose
    stationary_waypoint_.x = msg->pose.pose.position.x;
    stationary_waypoint_.y = msg->pose.pose.position.y;
    stationary_waypoint_.z = msg->pose.pose.position.z;
    stationary_waypoint_.desired_speed = 0.0;
    stationary_waypoint_.fixed_heading = true;
    stationary_waypoint_.heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
    stationary_waypoint_set_ = true;
}

void WaypointController::go_to_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check heading of the robot if it is not supposed to
    if (!waypoints_.front().fixed_heading)
    {
        // If heading is outside tolerance fix it before we start going toward the next waypoint
        // Desired heading is straight towards the waypoint
        float desired_heading = atan2(waypoints_.front().y - msg->pose.pose.position.y, waypoints_.front().x - msg->pose.pose.position.x);
        float heading_err = desired_heading - get_yaw_from_quaternion(msg->pose.pose.orientation);

        // Check for minimum signed angle
        if (heading_err > M_PI)
        {
            heading_err -= 2 * M_PI;
        }
        else if (heading_err < -M_PI)
        {
            heading_err += 2 * M_PI;
        }

        if (abs(heading_err) > heading_tolerance_)
        {
            // Rotate towards the correct heading, all other velocity fields are set to 0 by default
            geometry_msgs::msg::TwistStamped desired_velocity;
            desired_velocity.header.stamp = msg->header.stamp;
            desired_velocity.header.frame_id = msg->header.frame_id;
            // Since we are outside tolerance "max" speed is used
            desired_velocity.twist.angular.z = heading_err > 0 ? heading_speed_ : -heading_speed_;
            desired_velocity_pub_->publish(desired_velocity);
            return;
        }

        // If heading is within tolerance, go to waypoint
        float z_err = waypoints_.front().z - msg->pose.pose.position.z;
        geometry_msgs::msg::TwistStamped desired_velocity;
        desired_velocity.header.stamp = msg->header.stamp;
        desired_velocity.header.frame_id = msg->header.frame_id;
        desired_velocity.twist.linear.x = waypoints_.front().desired_speed;
        // TODO - Add a scaling system that takes size of error into consideration more than just linear scaling
        desired_velocity.twist.linear.z = z_err > 0 ? min(heave_speed_, heave_speed_ * z_err) : min(-heave_speed_*z_err, -heave_speed_);
        desired_velocity.twist.angular.z = heading_err > 0 ? min(heading_speed_, heading_speed_*heading_err) : min(-heading_speed_, -heading_speed_*heading_err);
        desired_velocity_pub_->publish(desired_velocity);
    }

    else
    {
        // Fixed heading implementation
        // Calculate position errors in world frame
        float x_err = waypoints_.front().x - msg->pose.pose.position.x;
        float y_err = waypoints_.front().y - msg->pose.pose.position.y;
        float z_err = waypoints_.front().z - msg->pose.pose.position.z;
        
        // Get current heading
        float current_heading = get_yaw_from_quaternion(msg->pose.pose.orientation);
        
        // Calculate heading error
        float heading_err = waypoints_.front().heading - current_heading;
        if (heading_err > M_PI) heading_err -= 2 * M_PI;
        if (heading_err < -M_PI) heading_err += 2 * M_PI;

        // Transform world frame errors to body frame
        float cos_h = cos(current_heading);
        float sin_h = sin(current_heading);
        float x_err_body = x_err * cos_h + y_err * sin_h;
        float y_err_body = -x_err * sin_h + y_err * cos_h;

        // Create velocity command
        geometry_msgs::msg::TwistStamped desired_velocity;
        desired_velocity.header.stamp = msg->header.stamp;
        desired_velocity.header.frame_id = msg->header.frame_id;

        // Scale velocities based on errors
        float speed = waypoints_.front().desired_speed;
        desired_velocity.twist.linear.x = x_err_body > 0 ? min(speed, speed * x_err_body) 
                                                       : max(-speed, -speed * abs(x_err_body));
        desired_velocity.twist.linear.y = y_err_body > 0 ? min(speed, speed * y_err_body) 
                                                       : max(-speed, -speed * abs(y_err_body));
        desired_velocity.twist.linear.z = z_err > 0 ? min(heave_speed_, heave_speed_ * z_err) 
                                                   : min(-heave_speed_, -heave_speed_ * z_err);
        
        // Maintain fixed heading
        desired_velocity.twist.angular.z = heading_err > 0 ? min(heading_speed_, heading_speed_ * heading_err) 
                                                         : max(-heading_speed_, -heading_speed_ * heading_err);

        RCLCPP_DEBUG(get_logger(), "Fixed heading control - errors (body frame): x=%.2f, y=%.2f, z=%.2f, h=%.2f",
                    x_err_body, y_err_body, z_err, heading_err);

        desired_velocity_pub_->publish(desired_velocity);

    }
}

void WaypointController::station_keep(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Check all degrees of freedom compared to the stationary waypoint and run correction
    float x_err_world = stationary_waypoint_.x - msg->pose.pose.position.x;
    float y_err_world = stationary_waypoint_.y - msg->pose.pose.position.y;
    float z_err = stationary_waypoint_.z - msg->pose.pose.position.z;
    float heading_err = stationary_waypoint_.heading - get_yaw_from_quaternion(msg->pose.pose.orientation);

    // Rotate x_err and y_err by taking current heading into consideration
    float x_err = x_err_world * cos(get_yaw_from_quaternion(msg->pose.pose.orientation)) + y_err_world * sin(get_yaw_from_quaternion(msg->pose.pose.orientation));
    float y_err = -x_err_world * sin(get_yaw_from_quaternion(msg->pose.pose.orientation)) + y_err_world * cos(get_yaw_from_quaternion(msg->pose.pose.orientation));

    // Station keep on the stationary waypoint, TODO Add scaling based on error more than just linear scaling
    geometry_msgs::msg::TwistStamped desired_velocity;
    desired_velocity.header.stamp = msg->header.stamp;
    desired_velocity.header.frame_id = msg->header.frame_id;
    desired_velocity.twist.linear.x = x_err > 0 ? min(station_keep_speed_, station_keep_speed_*x_err) : min(-station_keep_speed_, -station_keep_speed_*x_err);
    desired_velocity.twist.linear.y = y_err > 0 ? min(station_keep_speed_, station_keep_speed_*y_err) : min(-station_keep_speed_, -station_keep_speed_*y_err);
    desired_velocity.twist.linear.z = z_err > 0 ? min(heave_speed_, heave_speed_*z_err) : min(-heave_speed_, -heave_speed_*z_err);
    desired_velocity.twist.angular.z = heading_err > 0 ? min(heading_speed_, heading_speed_*heading_err) : min(-heading_speed_, -heading_speed_*heading_err);
    desired_velocity_pub_->publish(desired_velocity);
}

string WaypointController::waypoints_to_string()
{
    // Get list of waypoints on a string format
    int cnt = 1;
    ostringstream waypoints_string;
    for (const auto &waypoint : waypoints_)
    {
        waypoints_string << "Waypoint " + to_string(cnt) + "(x: " << waypoint.x
                         << ", y: " << waypoint.y
                         << ", z: " << waypoint.z
                         << ", desired_speed: " << waypoint.desired_speed
                         << ", fixed_heading: " << waypoint.fixed_heading
                         << ", heading: " << waypoint.heading << ")\n";
        cnt++;
    }
    return waypoints_string.str();
}

void WaypointController::setup_interfaces()
{
    // Desired Velocity Publisher
    desired_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(desired_velocity_topic_, 1);

    // Odometry Subscriber
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 1, bind(&WaypointController::controller_callback, this, placeholders::_1));

    // Waypoint Services
    add_waypoint_srv_ = create_service<mundus_mir_msgs::srv::AddWaypoint>(add_waypoint_srv_name_, bind(&WaypointController::add_waypoint, this, placeholders::_1, placeholders::_2));
    insert_waypoint_srv_ = create_service<mundus_mir_msgs::srv::InsertWaypoint>(insert_waypoint_srv_name_, bind(&WaypointController::insert_waypoint, this, placeholders::_1, placeholders::_2));
    remove_waypoint_srv_ = create_service<mundus_mir_msgs::srv::RemoveWaypoint>(remove_waypoint_srv_name_, bind(&WaypointController::remove_waypoint, this, placeholders::_1, placeholders::_2));
    get_waypoints_srv_ = create_service<mundus_mir_msgs::srv::GetWaypoints>(get_waypoints_srv_name_, bind(&WaypointController::get_waypoints, this, placeholders::_1, placeholders::_2));
    clear_waypoints_srv_ = create_service<mundus_mir_msgs::srv::ClearWaypoints>(clear_waypoints_srv_name_, bind(&WaypointController::clear_waypoints, this, placeholders::_1, placeholders::_2));
    go_to_waypoints_srv_ = create_service<mundus_mir_msgs::srv::GoToWaypoints>(go_to_waypoints_srv_name_, bind(&WaypointController::go_to_waypoints, this, placeholders::_1, placeholders::_2));
    get_waypoint_status_srv_ = create_service<mundus_mir_msgs::srv::GetWaypointStatus>(get_waypoint_status_srv_name_, bind(&WaypointController::get_waypoint_status, this, placeholders::_1, placeholders::_2));
    run_waypoint_controller_srv_ = create_service<mundus_mir_msgs::srv::RunWaypointController>(run_waypoint_controller_srv_name_, bind(&WaypointController::run_waypoint_controller, this, placeholders::_1, placeholders::_2));
}

void WaypointController::get_ros_parameters()
{
    // Declare parameters
    declare_parameter("odometry_topic", "/odom");
    declare_parameter("desired_vel_topic", "/desired_vel");
    declare_parameter("waypoint_service/add", "/blueye/add_waypoint");
    declare_parameter("waypoint_service/insert", "/blueye/insert_waypoint");
    declare_parameter("waypoint_service/remove", "/blueye/remove_waypoint");
    declare_parameter("waypoint_service/get", "/blueye/get_waypoints");
    declare_parameter("waypoint_service/clear", "/blueye/clear_waypoints");
    declare_parameter("waypoint_service/go_to", "/blueye/go_to_waypoints");
    declare_parameter("waypoint_service/get_waypoint_status", "/blueye/get_waypoint_status");
    declare_parameter("waypoint_service/run_waypoint_controller", "/blueye/run_waypoint_controller");
    declare_parameter("heading_tolerance", 0.3);
    declare_parameter("circle_of_acceptance", 0.5);
    declare_parameter("max_heading_rate", 0.1);
    declare_parameter("max_heave_rate", 0.3);
    declare_parameter("max_station_keep_rate", 0.3);

    // Get parameters
    get_parameter("odometry_topic", odometry_topic_);
    get_parameter("desired_vel_topic", desired_velocity_topic_);
    get_parameter("waypoint_service/add", add_waypoint_srv_name_);
    get_parameter("waypoint_service/insert", insert_waypoint_srv_name_);
    get_parameter("waypoint_service/remove", remove_waypoint_srv_name_);
    get_parameter("waypoint_service/get", get_waypoints_srv_name_);
    get_parameter("waypoint_service/clear", clear_waypoints_srv_name_);
    get_parameter("waypoint_service/go_to", go_to_waypoints_srv_name_);
    get_parameter("waypoint_service/get_waypoint_status", get_waypoint_status_srv_name_);
    get_parameter("waypoint_service/run_waypoint_controller", run_waypoint_controller_srv_name_);
    get_parameter("heading_tolerance", heading_tolerance_);
    get_parameter("circle_of_acceptance", circle_of_acceptance_);
    get_parameter("max_heading_rate", heading_speed_);
    get_parameter("max_heave_rate", heave_speed_);
    get_parameter("max_station_keep_rate", station_keep_speed_);
}

void WaypointController::add_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Request> request,
                                      std::shared_ptr<mundus_mir_msgs::srv::AddWaypoint::Response> response)
{
    // Add a new waypoint to the back of the waypoint list
    Waypoint new_waypoint;
    new_waypoint.x = request->x;
    new_waypoint.y = request->y;
    new_waypoint.z = request->z;
    new_waypoint.fixed_heading = request->fixed_heading;
    new_waypoint.heading = request->heading;

    if (!(request->desired_velocity >= 0.0))
    {
        new_waypoint.desired_speed = request->desired_velocity;
    }

    waypoints_.push_back(new_waypoint);

    // Set response
    response->accepted = true;
}

void WaypointController::insert_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::InsertWaypoint::Response> response)
{
    // Add a new waypoint and insert it at desired index
    Waypoint new_waypoint;
    new_waypoint.x = request->x;
    new_waypoint.y = request->y;
    new_waypoint.z = request->z;
    new_waypoint.fixed_heading = request->fixed_heading;
    new_waypoint.heading = request->heading;

    if (!(request->desired_velocity >= 0.0))
    {
        new_waypoint.desired_speed = request->desired_velocity;
    }
    int index = request->index > 0 ? request->index : 0;
    int amount_of_waypoints = waypoints_.size();
    if (index >= amount_of_waypoints)
    {
        waypoints_.push_back(new_waypoint);
    }
    else
    {
        waypoints_.insert(waypoints_.begin() + index, new_waypoint);
    }

    // Set response
    response->accepted = true;
}

void WaypointController::remove_waypoint(const std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::RemoveWaypoint::Response> response)
{
    // Remove waypoint at index
    int size_of_waypoints = waypoints_.size();
    if (size_of_waypoints < request->index || request->index < 0)
    {
        response->accepted = false;
        response->error_code = "Index out of bounds.";
        return;
    }
    waypoints_.erase(waypoints_.begin() + request->index);

    // Set response
    response->accepted = true;
}

void WaypointController::get_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Request> request,
                                       std::shared_ptr<mundus_mir_msgs::srv::GetWaypoints::Response> response)
{
    // Handle request to avoid warinings
    (void) request;
    // Set response
    response->accepted = true;
    response->waypoints = waypoints_to_string();
}

void WaypointController::clear_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::ClearWaypoints::Response> response)
{
    // Handle request to avoid warnings
    (void) request;
    // Clear all waypoints
    waypoints_.clear();

    // Set go_to_waypoint variable to false as we no longer have any waypoints
    go_to_waypoint_ = false;
    stationary_waypoint_set_ = false;

    // Set response
    response->accepted = true;
}

void WaypointController::go_to_waypoints(const std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Request> request,
                                         std::shared_ptr<mundus_mir_msgs::srv::GoToWaypoints::Response> response)
{
    // If no waypoints then we cant go to any
    if (waypoints_.size() == 0)
    {
        response->accepted = false;
        response->status_code = "No waypoints to go to.";
        return;
    }
    go_to_waypoint_ = request->run;
    stationary_waypoint_set_ = false;

    // Set response
    response->accepted = true;
}

void WaypointController::get_waypoint_status(const std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Request> request,
                                             std::shared_ptr<mundus_mir_msgs::srv::GetWaypointStatus::Response> response)
{
    // Handle request to avoid warnings
    (void) request;
    // Set response
    response->accepted = true;

    // TODO Figure out how we want to have this message written
    ostringstream status;
    status << "Controller running: " << run_controller_ << "\n";
    if (run_controller_) {
        status << "Amount of waypoints: " << waypoints_.size() << "\n";
        status << "Currently going to waypoint: " << go_to_waypoint_ << "\n";
        if (go_to_waypoint_)
        {
            status << "Current waypoint: " << waypoints_.front().x << ", " << waypoints_.front().y << ", " << waypoints_.front().z << "\n";
        }
        else {
            status << "Station Keep On Position: " << stationary_waypoint_.x << ", " << stationary_waypoint_.y << ", " << stationary_waypoint_.z << "\n";
        }
    }
    response->status_code = status.str();
}

void WaypointController::run_waypoint_controller(const std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Request> request,
                                                 std::shared_ptr<mundus_mir_msgs::srv::RunWaypointController::Response> response)
{
    // Set run_controller variable
    run_controller_ = request->run;

    // Set response
    response->accepted = true;
}