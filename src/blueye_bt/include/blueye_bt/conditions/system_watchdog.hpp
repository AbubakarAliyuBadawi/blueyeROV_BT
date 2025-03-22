// system_watchdog.hpp
#ifndef SYSTEM_WATCHDOG_HPP
#define SYSTEM_WATCHDOG_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <map>
#include <string>

class SystemWatchdog : public BT::ConditionNode {
private:
  struct TopicMonitor {
    std::string name;
    rclcpp::Time last_update;
    double timeout_seconds;
    bool is_critical;
    bool is_active;
  };

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, TopicMonitor> monitored_topics_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_force_sub_;
  
  // Reset mechanism
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  
  // State tracking
  bool system_healthy_ = true;
  int consecutive_failures_ = 0;
  int max_failures_before_reset_ = 3;
  rclcpp::Time last_reset_time_;
  double min_reset_interval_ = 60.0;  // Minimum seconds between resets
  
public:
  SystemWatchdog(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts() {
    return BT::PortsList({
      BT::InputPort<int>("max_failures", 3, "Maximum consecutive failures before triggering reset"),
      BT::InputPort<double>("min_reset_interval", 60.0, "Minimum seconds between reset attempts"),
      BT::OutputPort<std::string>("failing_topics", "List of topics that are failing")
    }); 
  }
  
  BT::NodeStatus tick() override;
  
private:
  // Topic callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdForceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  
  // Health check function
  bool checkSystemHealth(std::string& failing_topics);
  
  // Reset function
  bool triggerSystemReset();
};

#endif // SYSTEM_WATCHDOG_HPP