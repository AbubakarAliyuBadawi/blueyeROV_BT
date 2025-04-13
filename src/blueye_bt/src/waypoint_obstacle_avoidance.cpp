#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mundus_mir_msgs/srv/add_waypoint.hpp>
#include <mundus_mir_msgs/srv/clear_waypoints.hpp>
#include <mundus_mir_msgs/srv/go_to_waypoints.hpp>
#include <mundus_mir_msgs/srv/get_waypoint_status.hpp>
#include <vector>
#include <cstring>
#include <limits>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class WaypointObstacleAvoidance : public rclcpp::Node
{
public:
  WaypointObstacleAvoidance() : Node("waypoint_obstacle_avoidance"), is_avoiding_(false)
  {
    // Parameters
    this->declare_parameter("safety_distance", 5.0);  // meters
    this->declare_parameter("deviation_distance", 6.0); // meters
    this->declare_parameter("deviation_velocity", 0.1); // m/s
    this->declare_parameter("obstacle_avoidance_enabled", true);
    this->declare_parameter("waypoint_resume_delay", 0.1); // seconds
    
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    deviation_distance_ = this->get_parameter("deviation_distance").as_double();
    deviation_velocity_ = this->get_parameter("deviation_velocity").as_double();
    obstacle_avoidance_enabled_ = this->get_parameter("obstacle_avoidance_enabled").as_bool();
    waypoint_resume_delay_ = this->get_parameter("waypoint_resume_delay").as_double();
    
    // Create subscribers
    sonar_sub_ = this->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      "/mundus_mir/sonar", 10, 
      std::bind(&WaypointObstacleAvoidance::sonarCallback, this, std::placeholders::_1));
      
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/blueye/odometry_frd/gt", 10,
      std::bind(&WaypointObstacleAvoidance::odomCallback, this, std::placeholders::_1));
      
    // Create publisher for status
    avoidance_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_avoidance/active", 10);
    
    // Create service clients for waypoint control
    add_waypoint_client_ = this->create_client<mundus_mir_msgs::srv::AddWaypoint>("/blueye/add_waypoint");
    clear_waypoints_client_ = this->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    go_to_waypoints_client_ = this->create_client<mundus_mir_msgs::srv::GoToWaypoints>("/blueye/go_to_waypoints");
    get_status_client_ = this->create_client<mundus_mir_msgs::srv::GetWaypointStatus>("/blueye/get_waypoint_status");
    
    // Create enable/disable service
    enable_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/obstacle_avoidance/enable",
      std::bind(&WaypointObstacleAvoidance::enableAvoidanceCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // Timer for periodic status checks
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&WaypointObstacleAvoidance::checkWaypointStatus, this));
    
    RCLCPP_INFO(this->get_logger(), "Waypoint Obstacle Avoidance node initialized");
  }

private:
  void sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg)
  {
    if (!obstacle_avoidance_enabled_ || !have_pose_) {
      return;
    }
    
    // Only process if we're not already avoiding
    if (is_avoiding_ || is_recovering_) {
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
    
    // Check if obstacle is detected in the center sector
    if (min_dist_center_ < safety_distance_) {
      // Obstacle detected, get current waypoint status before taking action
      getWaypointStatus();
    }
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    have_pose_ = true;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
      current_pose_.orientation.x,
      current_pose_.orientation.y,
      current_pose_.orientation.z,
      current_pose_.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
  }
  
  void getWaypointStatus()
  {
    auto request = std::make_shared<mundus_mir_msgs::srv::GetWaypointStatus::Request>();
    
    auto response_callback = [this](
        rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedFuture future) {
      auto response = future.get();
      if (response->accepted) {
        parseWaypointStatus(response->status_code);
      }
    };
    
    get_status_client_->async_send_request(request, response_callback);
  }
  
void parseWaypointStatus(const std::string& status) {
    // Extract information from status string
    bool is_going_to_waypoint = status.find("Currently going to waypoint: 1") != std::string::npos;
    bool has_waypoints = status.find("Amount of waypoints: 0") == std::string::npos;
    
    // Extract current waypoint coordinates if available
    size_t wp_pos = status.find("Current waypoint: x=");
    if (wp_pos != std::string::npos && is_going_to_waypoint && has_waypoints) {
        // Parse current waypoint coordinates from string
        std::string wp_str = status.substr(wp_pos);
        float x, y, z;
        sscanf(wp_str.c_str(), "Current waypoint: x=%f, y=%f, z=%f", &x, &y, &z);
        
        // Store original waypoint
        original_waypoint_.x = x;
        original_waypoint_.y = y;
        original_waypoint_.z = z;  // Store the original z-value
        have_original_waypoint_ = true;
        
        // If obstacle detected but we're not avoiding yet, start avoidance
        if (min_dist_center_ < safety_distance_ && !is_avoiding_ && !is_recovering_) {
            startObstacleAvoidance();
        }
    }
}
  
  void startObstacleAvoidance()
  {
    RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f meters. Starting avoidance maneuver.", min_dist_center_);
    
    is_avoiding_ = true;
    avoidance_start_time_ = this->now();
    
    // Create deviation waypoint
    float avoidance_x, avoidance_y;
    
    // Determine if we should go left or right
    bool go_left = min_dist_left_ <= min_dist_right_;
    
    // Calculate 25-degree direction instead of 45 degrees
    float deviation_angle = current_yaw_ + (go_left ? (25.0 * M_PI/180.0) : -(25.0 * M_PI/180.0));

    // Calculate deviation point
    avoidance_x = current_pose_.position.x + deviation_distance_ * cos(deviation_angle);
    avoidance_y = current_pose_.position.y + deviation_distance_ * sin(deviation_angle);
    
    RCLCPP_INFO(this->get_logger(), "Creating avoidance waypoint at x=%.2f, y=%.2f (going %s)",
               avoidance_x, avoidance_y, go_left ? "left" : "right");
    
    // Clear current waypoints
    auto clear_request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    clear_request->clear = true;
    
    auto clear_response_callback = [this, avoidance_x, avoidance_y](
        rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedFuture future) {
      auto response = future.get();
      if (response->accepted) {
        // Add deviation waypoint
        addDeviationWaypoint(avoidance_x, avoidance_y);
      }
    };
    
    clear_waypoints_client_->async_send_request(clear_request, clear_response_callback);
    
    // Publish avoidance status
    auto status_msg = std::make_unique<std_msgs::msg::Bool>();
    status_msg->data = true;
    avoidance_status_pub_->publish(std::move(status_msg));
  }
  
  void addDeviationWaypoint(float x, float y) {
    auto add_request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
    add_request->x = x;
    add_request->y = y;
    add_request->z = original_waypoint_.z;  // Use original waypoint's z-value
    add_request->desired_velocity = deviation_velocity_;
    add_request->fixed_heading = false;  // Look toward waypoint
    add_request->heading = 0.0;  // Not used with fixed_heading=false
    
    auto add_response_callback = [this](
        rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedFuture future) {
        auto response = future.get();
        if (response->accepted) {
            // Start going to waypoint
            auto go_request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
            go_request->run = true;
            
            auto go_response_callback = [this](
                rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedFuture future) {
                // Successfully started going to avoidance waypoint
            };
            
            go_to_waypoints_client_->async_send_request(go_request, go_response_callback);
        }
    };
    
    add_waypoint_client_->async_send_request(add_request, add_response_callback);
}
  
  void checkWaypointStatus()
  {
    if (!is_avoiding_ && !is_recovering_) {
      return;
    }
    
    // If we're avoiding, check if we've reached the avoidance waypoint
    if (is_avoiding_) {
      float dist_to_waypoint = std::sqrt(
        std::pow(current_pose_.position.x - avoidance_waypoint_.x, 2) +
        std::pow(current_pose_.position.y - avoidance_waypoint_.y, 2));
      
      // If we've reached the avoidance waypoint or timeout, start recovery
      if (dist_to_waypoint < 1.0 || 
          (this->now() - avoidance_start_time_).seconds() > 15.0) {
        startRecovery();
      }
    }
    
    // If we're recovering, check if it's time to return to original waypoint
    if (is_recovering_) {
      if ((this->now() - recovery_start_time_).seconds() > waypoint_resume_delay_) {
        returnToOriginalWaypoint();
      }
    }
  }
  
  void startRecovery()
  {
    is_avoiding_ = false;
    is_recovering_ = true;
    recovery_start_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Avoidance waypoint reached, starting recovery phase");
  }
  
  void returnToOriginalWaypoint()
  {
    RCLCPP_INFO(this->get_logger(), "Recovery complete, returning to original waypoint");
    
    is_recovering_ = false;
    
    // Clear current waypoints
    auto clear_request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    clear_request->clear = true;
    
    auto clear_response_callback = [this](
        rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedFuture future) {
      auto response = future.get();
      if (response->accepted && have_original_waypoint_) {
        // Add original waypoint back
        auto add_request = std::make_shared<mundus_mir_msgs::srv::AddWaypoint::Request>();
        add_request->x = original_waypoint_.x;
        add_request->y = original_waypoint_.y;
        add_request->z = original_waypoint_.z;
        add_request->desired_velocity = original_waypoint_.velocity;
        add_request->fixed_heading = original_waypoint_.fixed_heading;
        add_request->heading = original_waypoint_.heading;
        
        add_waypoint_client_->async_send_request(add_request, 
            [this](rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedFuture future) {
          auto response = future.get();
          if (response->accepted) {
            // Start going to waypoint
            auto go_request = std::make_shared<mundus_mir_msgs::srv::GoToWaypoints::Request>();
            go_request->run = true;
            go_to_waypoints_client_->async_send_request(go_request);
            
            // Reset avoidance status
            auto status_msg = std::make_unique<std_msgs::msg::Bool>();
            status_msg->data = false;
            avoidance_status_pub_->publish(std::move(status_msg));
          }
        });
      }
    };
    
    clear_waypoints_client_->async_send_request(clear_request, clear_response_callback);
  }
  
  void enableAvoidanceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    obstacle_avoidance_enabled_ = request->data;
    if (obstacle_avoidance_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Obstacle avoidance enabled");
    } else {
      is_avoiding_ = false;
      is_recovering_ = false;
      RCLCPP_INFO(this->get_logger(), "Obstacle avoidance disabled");
      
      // Publish disabled status
      auto status_msg = std::make_unique<std_msgs::msg::Bool>();
      status_msg->data = false;
      avoidance_status_pub_->publish(std::move(status_msg));
    }
    
    response->success = true;
    response->message = obstacle_avoidance_enabled_ ? 
                        "Obstacle avoidance enabled" : 
                        "Obstacle avoidance disabled";
  }
  
  float findMinDistanceInSector(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg,
                               int sector_start, int sector_end)
  {
    float min_distance = std::numeric_limits<float>::max();
    float intensity_threshold = 0.01f;  // Adjust based on your sonar's characteristics
    
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
          min_distance = std::min(min_distance, range);
          break;  // Found closest obstacle in this beam
        }
      }
    }
    
    return (min_distance == std::numeric_limits<float>::max()) ? 
            msg->ranges.back() : min_distance;  // Return max range if no obstacle detected
  }
  
  // Waypoint data structure
  struct Waypoint {
    float x;
    float y;
    float z;
    float velocity;
    bool fixed_heading;
    float heading;
  };
  
  // Parameters
  double safety_distance_;
  double deviation_distance_;
  double deviation_velocity_;
  double waypoint_resume_delay_;
  bool obstacle_avoidance_enabled_;
  
  // State variables
  bool is_avoiding_ = false;
  bool is_recovering_ = false;
  bool have_pose_ = false;
  bool have_original_waypoint_ = false;
  float min_dist_left_;
  float min_dist_center_;
  float min_dist_right_;
  float current_yaw_;
  Waypoint original_waypoint_;
  Waypoint avoidance_waypoint_;
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::Time avoidance_start_time_;
  rclcpp::Time recovery_start_time_;
  
  // ROS interfaces
  rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoidance_status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  
  // Service clients
  rclcpp::Client<mundus_mir_msgs::srv::AddWaypoint>::SharedPtr add_waypoint_client_;
  rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_waypoints_client_;
  rclcpp::Client<mundus_mir_msgs::srv::GoToWaypoints>::SharedPtr go_to_waypoints_client_;
  rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr get_status_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}