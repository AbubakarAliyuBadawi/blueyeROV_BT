// camera_condition.cpp
#include "blueye_bt/conditions/camera_condition.hpp"

CheckCameraStatus::CheckCameraStatus(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config),
    camera_ok_(false) {
  
  node_ = rclcpp::Node::make_shared("camera_check_node");
  
  // Get timeout from port using the reference method of getInput
  double timeout = 2.0;  // Default value
  if (getInput("timeout_seconds", timeout)) {
    timeout_seconds_ = timeout;
  }
  
  camera_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/blueye/camera_1/image_raw", 10,
    std::bind(&CheckCameraStatus::cameraCallback, this, std::placeholders::_1));
    
  // Initialize the last message time
  last_msg_time_ = node_->now();
}

BT::NodeStatus CheckCameraStatus::tick() {
  rclcpp::spin_some(node_);
  
  // Check if we've received camera messages recently
  auto time_since_last_msg = (node_->now() - last_msg_time_).seconds();
  
  if (camera_ok_ && time_since_last_msg < timeout_seconds_) {
    RCLCPP_DEBUG(node_->get_logger(), "Camera working properly. Last message %.2f seconds ago", time_since_last_msg);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Camera issue detected. No valid data for %.2f seconds", time_since_last_msg);
    return BT::NodeStatus::FAILURE;
  }
}

void CheckCameraStatus::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  // Update last message time
  last_msg_time_ = node_->now();
  
  // Check if image data is valid
  camera_ok_ = (msg->width > 0 && msg->height > 0 && !msg->data.empty());
  
  // Optional: Additional checks could be added here
  // For example, checking for minimum image dimensions or checking if the image is not all black
}