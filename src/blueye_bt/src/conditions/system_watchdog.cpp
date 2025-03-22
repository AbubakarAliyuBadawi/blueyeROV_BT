// system_watchdog.cpp
#include "blueye_bt/conditions/system_watchdog.hpp"

SystemWatchdog::SystemWatchdog(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {
  
  node_ = rclcpp::Node::make_shared("system_watchdog_node");
  
  // Get configuration from ports using the reference method of getInput
  int failures = 3;  // Default value
  if (getInput("max_failures", failures)) {
    max_failures_before_reset_ = failures;
  }

  double interval = 60.0;  // Default value
  if (getInput("min_reset_interval", interval)) {
    min_reset_interval_ = interval;
  }
  
  // Initialize the monitored topics
  rclcpp::Time now = node_->now();
  
  monitored_topics_["/blueye/odometry_frd/gt"] = {
    "Odometry", now, 1.0, true, false
  };
  
  monitored_topics_["/blueye/cmd_force"] = {
    "Command Force", now, 3.0, true, false
  };
  
  // Initialize subscribers
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/blueye/odometry_frd/gt", 10,
    std::bind(&SystemWatchdog::odomCallback, this, std::placeholders::_1));
    
  cmd_force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/blueye/cmd_force", 10,
    std::bind(&SystemWatchdog::cmdForceCallback, this, std::placeholders::_1));
  
  // Initialize reset client
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/blueye/reset_system");
  
  // Initialize reset time
  last_reset_time_ = now;
  
  RCLCPP_INFO(node_->get_logger(), "System watchdog initialized");
}

BT::NodeStatus SystemWatchdog::tick() {
  rclcpp::spin_some(node_);
  
  std::string failing_topics;
  bool is_healthy = checkSystemHealth(failing_topics);
  
  // Set output port with list of failing topics
  setOutput("failing_topics", failing_topics);
  
  if (is_healthy) {
    consecutive_failures_ = 0;
    system_healthy_ = true;
    return BT::NodeStatus::SUCCESS;
  } else {
    consecutive_failures_++;
    
    if (consecutive_failures_ >= max_failures_before_reset_) {
      // Check if enough time has passed since last reset
      double time_since_last_reset = (node_->now() - last_reset_time_).seconds();
      
      if (time_since_last_reset > min_reset_interval_) {
        RCLCPP_WARN(node_->get_logger(), 
                   "System unhealthy for %d consecutive checks. Triggering reset...",
                   consecutive_failures_);
        
        if (triggerSystemReset()) {
          consecutive_failures_ = 0;
          last_reset_time_ = node_->now();
        }
      } else {
        RCLCPP_WARN(node_->get_logger(), 
                   "System needs reset but waiting for cooldown period. %.1f seconds remaining.",
                   min_reset_interval_ - time_since_last_reset);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), 
                 "System unhealthy: %s. Failure count: %d/%d",
                 failing_topics.c_str(), consecutive_failures_, max_failures_before_reset_);
    }
    
    system_healthy_ = false;
    return BT::NodeStatus::FAILURE;
  }
}

bool SystemWatchdog::checkSystemHealth(std::string& failing_topics) {
  rclcpp::Time now = node_->now();
  bool all_critical_active = true;
  failing_topics = "";
  
  for (const auto& [topic, monitor] : monitored_topics_) {
    double elapsed = (now - monitor.last_update).seconds();
    bool topic_active = monitor.is_active && (elapsed < monitor.timeout_seconds);
    
    if (!topic_active) {
      if (!failing_topics.empty()) {
        failing_topics += ", ";
      }
      
      if (monitor.is_active) {
        failing_topics += monitor.name + " (timeout: " + std::to_string(elapsed) + "s)";
      } else {
        failing_topics += monitor.name + " (no data)";
      }
      
      if (monitor.is_critical) {
        all_critical_active = false;
      }
    }
  }
  
  return all_critical_active;
}

bool SystemWatchdog::triggerSystemReset() {
  if (!reset_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Reset service not available");
    return false;
  }
  
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = reset_client_->async_send_request(request);
  
  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "System reset triggered successfully");
      return true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Reset failed: %s", response->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Reset service call timed out");
    return false;
  }
}

void SystemWatchdog::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto& topic = monitored_topics_["/blueye/odometry_frd/gt"];
  topic.last_update = node_->now();
  topic.is_active = true;
}

void SystemWatchdog::cmdForceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  auto& topic = monitored_topics_["/blueye/cmd_force"];
  topic.last_update = node_->now();
  topic.is_active = true;
}