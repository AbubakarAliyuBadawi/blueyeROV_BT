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
  
  // Initialize the monitored topics
  rclcpp::Time now = node_->now();
  
  monitored_topics_["/blueye/odometry_frd/gt"] = {
    "Odometry", now, 10.0, true, false
  };
  
  monitored_topics_["/blueye/cmd_force"] = {
    "Command Force", now, 10.0, true, false
  };
  
  // Initialize subscribers
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/blueye/odometry_frd/gt", 10,
    std::bind(&SystemWatchdog::odomCallback, this, std::placeholders::_1));
    
  cmd_force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/blueye/cmd_force", 10,
    std::bind(&SystemWatchdog::cmdForceCallback, this, std::placeholders::_1));
  
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
    
    // REMOVE THE RESET LOGIC - Keep only the warning log
    RCLCPP_WARN(node_->get_logger(), 
               "System unhealthy: %s. Failure count: %d/%d",
               failing_topics.c_str(), consecutive_failures_, max_failures_before_reset_);
    
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