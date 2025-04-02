// sonar_condition.cpp
#include "blueye_bt_alt/conditions/sonar_condition.hpp"

CheckSonarStatus::CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config), sonar_ok_(false) {
  
  node_ = rclcpp::Node::make_shared("sonar_check_node");
  
  // Get configuration from ports using the reference method of getInput
  double timeout = 3.0;  // Default value
  if (getInput("timeout_seconds", timeout)) {
    timeout_seconds_ = timeout;
  }

  int min_beams = 10;  // Default value
  if (getInput("min_beam_count", min_beams)) {
    min_beam_count_ = min_beams;
  }
  
  sonar_sub_ = node_->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
    "/mundus_mir/sonar", 10,
    std::bind(&CheckSonarStatus::sonarCallback, this, std::placeholders::_1));
    
  // Initialize the last message time
  last_msg_time_ = node_->now();
}

BT::NodeStatus CheckSonarStatus::tick() {
  rclcpp::spin_some(node_);
  
  // Check if we've received sonar messages recently
  auto time_since_last_msg = (node_->now() - last_msg_time_).seconds();
  
  if (sonar_ok_ && time_since_last_msg < timeout_seconds_) {
    RCLCPP_DEBUG(node_->get_logger(), 
                "Sonar working properly. Last message %.2f seconds ago", 
                time_since_last_msg);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), 
               "Sonar issue detected. No valid data for %.2f seconds", 
               time_since_last_msg);
    return BT::NodeStatus::FAILURE;
  }
}

void CheckSonarStatus::sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
  // Update last message time
  last_msg_time_ = node_->now();
  
  // Check if sonar data is valid
  bool has_beam_directions = !msg->beam_directions.empty() && 
                             msg->beam_directions.size() >= min_beam_count_;
  bool has_ranges = !msg->ranges.empty();
  bool has_image_data = !msg->image.data.empty();
  
  // Consider the sonar working if it has all the expected components
  sonar_ok_ = has_beam_directions && has_ranges && has_image_data;
  
  if (!sonar_ok_) {
    RCLCPP_WARN(node_->get_logger(), 
               "Received invalid sonar data: beam_directions=%d, ranges=%d, image_data=%d",
               has_beam_directions, has_ranges, has_image_data);
  }
}