// system_watchdog.hpp
#ifndef SYSTEM_WATCHDOG_HPP
#define SYSTEM_WATCHDOG_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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
  
  // State tracking
  bool system_healthy_ = true;
  int consecutive_failures_ = 0;
  int max_failures_before_reset_ = 3;
  
public:
  SystemWatchdog(const std::string& name, const BT::NodeConfiguration& config);
  
  static BT::PortsList providedPorts() {
    return BT::PortsList({
      BT::InputPort<int>("max_failures", 3, "Maximum consecutive failures before logging warning"),
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
};

#endif // SYSTEM_WATCHDOG_HPP