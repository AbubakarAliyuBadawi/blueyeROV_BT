#include "rov_mission_bt/managers/mission_manager.hpp"
#include "rov_mission_bt/behaviors/navigate_to_waypoint.hpp"
#include "rov_mission_bt/behaviors/station_keeping.hpp"
#include "rov_mission_bt/conditions/battery_condition.hpp"

MissionManager::MissionManager() 
    : Node("mission_manager"),
      mission_running_(false),
      mission_should_stop_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(0.0),
      current_heading_(0.0) {
    
    // Declare parameters for mission BT file paths
    this->declare_parameter("pipeline_mission_file", "");
    this->declare_parameter("lawnmower_mission_file", "");
    this->declare_parameter("station_keeping_duration", 20);  // seconds
    
    // Get the file paths for mission behavior trees
    pipeline_mission_file_ = this->get_parameter("pipeline_mission_file").as_string();
    lawnmower_mission_file_ = this->get_parameter("lawnmower_mission_file").as_string();
    station_keeping_duration_ = this->get_parameter("station_keeping_duration").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Pipeline mission file: %s", pipeline_mission_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Lawnmower mission file: %s", lawnmower_mission_file_.c_str());
    
    // Create service for mission switching
    mission_service_ = this->create_service<mundus_mir_msgs::srv::MissionManager>(
        "/blueye/mission_manager",
        std::bind(&MissionManager::handle_mission_switch, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // Create clients for waypoint services
    waypoint_controller_client_ = this->create_client<mundus_mir_msgs::srv::RunWaypointController>(
        "/blueye/run_waypoint_controller");
    clear_waypoints_client_ = this->create_client<mundus_mir_msgs::srv::ClearWaypoints>(
        "/blueye/clear_waypoints");
    
    // Create odometry subscriber to track position for safe state transition
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/blueye/odom", 10, 
        std::bind(&MissionManager::odom_callback, this, std::placeholders::_1));
    
    // Initialize the BT factory
    init_bt_factory();
    
    // Start with no active mission
    current_mission_ = "none";
    active_tree_ = nullptr;
    mission_thread_ = nullptr;
    
    RCLCPP_INFO(this->get_logger(), "Mission Manager initialized and ready");
}

MissionManager::~MissionManager() {
    stop_current_mission();
}

void MissionManager::init_bt_factory() {
    // Register your existing node types here (same as in your main.cpp)
    factory_.registerNodeType<BT::RetryNode>("RetryNode");
    factory_.registerNodeType<BT::SequenceNode>("SequenceNode");
    
    // Register NavigateToWaypoint
    factory_.registerBuilder<NavigateToWaypoint>(
        "NavigateToWaypoint",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<NavigateToWaypoint>(name, config);
        });

    // Register StationKeeping
    factory_.registerBuilder<StationKeeping>(
        "StationKeeping",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<StationKeeping>(name, config);
        });

    // Register CheckBatteryLevel
    factory_.registerBuilder<CheckBatteryLevel>(
        "CheckBatteryLevel",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<CheckBatteryLevel>(name, config);
        });
        
    // Add any other custom nodes from your main.cpp
}

void MissionManager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;
    
    // Extract yaw from quaternion
    auto q = msg->pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    current_heading_ = std::atan2(siny_cosp, cosy_cosp);
}

void MissionManager::handle_mission_switch(
    const std::shared_ptr<mundus_mir_msgs::srv::MissionManager::Request> request,
    std::shared_ptr<mundus_mir_msgs::srv::MissionManager::Response> response) {
    
    std::string requested_mission = request->mission_type;
    bool force_switch = request->force_switch;
    
    RCLCPP_INFO(this->get_logger(), "Received mission switch request to: %s", requested_mission.c_str());
    
    // Check if the requested mission is already running
    if (requested_mission == current_mission_) {
        response->success = true;
        response->current_mission = current_mission_;
        response->message = "Mission already running";
        return;
    }
    
    // Stop the current mission and transition to a safe state
    if (current_mission_ != "none") {
        RCLCPP_INFO(this->get_logger(), "Stopping current mission: %s", current_mission_.c_str());
        
        if (!force_switch) {
            // Transition to safe state (station keeping)
            if (!transition_to_safe_state()) {
                response->success = false;
                response->current_mission = current_mission_;
                response->message = "Failed to transition to safe state";
                return;
            }
        }
        
        // Stop the current mission
        stop_current_mission();
    }
    
    // Start the requested mission
    bool success = false;
    if (requested_mission == "pipeline") {
        success = start_pipeline_mission();
    } else if (requested_mission == "lawnmower") {
        success = start_lawnmower_mission();
    } else if (requested_mission == "none") {
        // Just stop the current mission
        success = true;
    } else {
        response->success = false;
        response->current_mission = "none";
        response->message = "Unknown mission type: " + requested_mission;
        return;
    }
    
    if (success) {
        current_mission_ = requested_mission;
        response->success = true;
        response->current_mission = current_mission_;
        response->message = "Mission switched successfully";
    } else {
        response->success = false;
        response->current_mission = "none";
        response->message = "Failed to start mission: " + requested_mission;
    }
}

bool MissionManager::transition_to_safe_state() {
    RCLCPP_INFO(this->get_logger(), "Transitioning to safe state (station keeping)");
    
    // Stop the current mission but don't update current_mission_ yet
    mission_should_stop_ = true;
    
    // Wait for the mission to stop
    {
        std::unique_lock<std::mutex> lock(mission_mutex_);
        mission_cv_.wait(lock, [this]{ return !mission_running_; });
    }
    
    // Get current position from last odometry message
    double x, y, z, heading;
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        x = current_x_;
        y = current_y_;
        z = current_z_;
        heading = current_heading_;
    }
    
    // Clear any existing waypoints
    auto clear_request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    clear_request->clear = true;
    
    if (!clear_waypoints_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Clear waypoints service not available");
        return false;
    }
    
    auto clear_future = clear_waypoints_client_->async_send_request(clear_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), clear_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear waypoints");
        return false;
    }
    
    // Use the station keeping behavior
    // Note: In a real implementation, you might want to call the station keeping service
    // instead of directly creating a behavior tree
    
    // For now, we'll assume the ROV will stay in position after stopping mission
    RCLCPP_INFO(this->get_logger(), "Safe state transition complete");
    return true;
}

void MissionManager::stop_current_mission() {
    mission_should_stop_ = true;
    
    if (mission_thread_ != nullptr) {
        mission_thread_->join();
        delete mission_thread_;
        mission_thread_ = nullptr;
    }
    
    active_tree_.reset();
    mission_running_ = false;
    mission_should_stop_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Mission stopped");
}

bool MissionManager::start_pipeline_mission() {
    RCLCPP_INFO(this->get_logger(), "Starting pipeline mission");
    return start_mission(pipeline_mission_file_);
}

bool MissionManager::start_lawnmower_mission() {
    RCLCPP_INFO(this->get_logger(), "Starting lawnmower mission");
    return start_mission(lawnmower_mission_file_);
}

bool MissionManager::start_mission(const std::string& mission_file) {
    try {
        // Create the behavior tree
        active_tree_ = std::make_shared<BT::Tree>(factory_.createTreeFromFile(mission_file));
        
        // Start the mission in a separate thread
        mission_thread_ = new std::thread([this]() {
            mission_running_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Mission thread started");
            
            // Create a logger for visualization
            BT::Groot2Publisher publisher(*active_tree_, 6677);
            
            const auto sleep_ms = std::chrono::milliseconds(100);
            auto status = BT::NodeStatus::RUNNING;
            
            while (!mission_should_stop_ && status == BT::NodeStatus::RUNNING) {
                status = active_tree_->tickOnce();
                std::this_thread::sleep_for(sleep_ms);
            }
            
            active_tree_->haltTree();
            
            {
                std::lock_guard<std::mutex> lock(mission_mutex_);
                mission_running_ = false;
            }
            mission_cv_.notify_all();
            
            RCLCPP_INFO(this->get_logger(), "Mission thread completed");
        });
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start mission: %s", e.what());
        return false;
    }
}