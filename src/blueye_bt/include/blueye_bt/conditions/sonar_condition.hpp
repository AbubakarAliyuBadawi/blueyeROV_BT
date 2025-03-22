// sonar_condition.hpp
#ifndef SONAR_CONDITION_HPP
#define SONAR_CONDITION_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>

class CheckSonarStatus : public BT::ConditionNode {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_sub_;
  
  bool sonar_ok_ = false;
  rclcpp::Time last_msg_time_;
  double timeout_seconds_ = 2.0;  // How long without messages before considering the sonar faulty
  int min_beam_count_ = 10;       // Minimum number of beams expected

public:
  CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config);
  
  // Port definition
  static BT::PortsList providedPorts() {
    return BT::PortsList({
      BT::InputPort<double>("timeout_seconds", 2.0, "Maximum time without sonar messages before reporting failure"),
      BT::InputPort<int>("min_beam_count", 10, "Minimum number of beams expected in sonar data")
    }); 
  }
  
  BT::NodeStatus tick() override;
  
private:
  void sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);
};

#endif // SONAR_CONDITION_HPP