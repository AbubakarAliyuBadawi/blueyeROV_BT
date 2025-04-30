#include "blueye_bt_real/behaviors/activate_auto_modes.hpp"
#include <chrono>

ActivateAutoModes::ActivateAutoModes(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("activate_auto_modes");
    depth_client_ = node_->create_client<std_srvs::srv::SetBool>("/blueye/depth_hold");
    heading_client_ = node_->create_client<std_srvs::srv::SetBool>("/blueye/heading_hold");
}

BT::NodeStatus ActivateAutoModes::onStart()
{
    bool depth_hold = true;
    bool heading_hold = true;
    
    // Get values from ports
    getInput("depth_hold", depth_hold);
    getInput("heading_hold", heading_hold);
    getInput("hold_duration", hold_duration_);
    
    RCLCPP_INFO(node_->get_logger(), "Setting auto modes - Depth hold: %s, Heading hold: %s for %d seconds", 
                depth_hold ? "true" : "false", heading_hold ? "true" : "false", hold_duration_);

    // Call the services
    bool depth_success = callAutoService(depth_client_, depth_hold, "depth hold");
    bool heading_success = callAutoService(heading_client_, heading_hold, "heading hold");
    
    services_activated_ = depth_success && heading_success;
    
    if (!services_activated_) {
        return BT::NodeStatus::FAILURE;
    }
    
    // Record start time
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActivateAutoModes::onRunning()
{
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    
    // Report progress periodically
    if (elapsed_seconds % 2 == 0) {  // Log every 2 seconds
        RCLCPP_INFO(node_->get_logger(), "Maintaining position... %ld/%d seconds completed", 
                   elapsed_seconds, hold_duration_);
    }
    
    // Check if we've held position long enough
    if (elapsed_seconds >= hold_duration_) {
        RCLCPP_INFO(node_->get_logger(), "Successfully maintained position for %d seconds", hold_duration_);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;
}

void ActivateAutoModes::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "Position hold interrupted!");
    
    // Optionally disable the modes when interrupted
    // callAutoService(depth_client_, false, "depth hold");
    // callAutoService(heading_client_, false, "heading hold");
}

bool ActivateAutoModes::callAutoService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, 
                                     bool enable, const std::string& service_name)
{
    // Wait for service to be available
    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "%s service not available after waiting", service_name.c_str());
        return false;
    }
    
    // Create and send request
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    
    auto future_result = client->async_send_request(request);
    
    // Wait for response
    if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(2)) 
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call %s service", service_name.c_str());
        return false;
    }
    
    auto response = future_result.get();
    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service call to %s failed: %s", 
                    service_name.c_str(), response->message.c_str());
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "%s set to %s: %s", 
                service_name.c_str(), enable ? "active" : "inactive", response->message.c_str());
    return true;
}