#include "blueye_bt/actions/altitude_control_action.hpp"
#include <chrono>
#include <rclcpp/parameter_client.hpp>

using namespace std::chrono_literals;

AltitudeControlAction::AltitudeControlAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
    
    // Create client for the enable service
    enable_client_ = g_node->create_client<std_srvs::srv::SetBool>("/altitude_controller/enable");
    
    // Create parameter client
    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(g_node, "altitude_controller");
    
    // Check if client was created successfully
    if (!enable_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Altitude controller service not available");
    }
    
    if (!param_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(g_node->get_logger(), "Altitude controller parameter service not available");
    }
}

BT::NodeStatus AltitudeControlAction::tick() {
    // Get input values
    bool enable = true;
    double target_altitude = 2.0;
    
    getInput("enable", enable);
    getInput("target_altitude", target_altitude);
    
    // First set the target altitude parameter
    if (enable) {
        // Make sure parameter client is ready
        if (!param_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(g_node->get_logger(), "Altitude controller parameter service not available");
            return BT::NodeStatus::FAILURE;
        }
        
        // Set parameter on the altitude controller node
        auto result = param_client_->set_parameters({
            rclcpp::Parameter("target_altitude", target_altitude)
        });
        
        if (!result[0].successful) {
            RCLCPP_ERROR(g_node->get_logger(), "Failed to set target_altitude parameter: %s", 
                        result[0].reason.c_str());
            return BT::NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(g_node->get_logger(), "Setting target altitude to %.2f meters", target_altitude);
    }
    
    // Then enable/disable the controller
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    
    auto future = enable_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(g_node, future, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call altitude controller service");
        return BT::NodeStatus::FAILURE;
    }
    
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(g_node->get_logger(), "Altitude controller service call failed: %s",
                   response->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Altitude control %s successfully",
               enable ? "enabled" : "disabled");
    return BT::NodeStatus::SUCCESS;
}