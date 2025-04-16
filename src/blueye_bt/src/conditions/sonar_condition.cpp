#include "blueye_bt/conditions/sonar_condition.hpp"

CheckSonarStatus::CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), sonar_ok_(true) {
    
    node_ = rclcpp::Node::make_shared("sonar_check_node");
    
    // Get configuration from ports using the reference method of getInput
    double timeout = 10.0; // Increased default value for development
    if (getInput("timeout_seconds", timeout)) {
        timeout_seconds_ = timeout;
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
    
    // Consider the sonar working as long as we're receiving messages
    // No content validation - if we get a message, sonar is considered working
    sonar_ok_ = true;
    
    // Optional debug logging
    RCLCPP_DEBUG(node_->get_logger(), 
                "Received sonar data. Topic is active.");
}