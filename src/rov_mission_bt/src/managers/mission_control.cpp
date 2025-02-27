#include "rov_mission_bt/managers/mission_control.hpp"
#include <iostream>

MissionControlClient::MissionControlClient() : Node("mission_control_client") {
    client_ = this->create_client<mundus_mir_msgs::srv::MissionManager>("/blueye/mission_manager");
}

bool MissionControlClient::switch_mission(const std::string& mission_type, bool force) {
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Mission manager service not available");
        return false;
    }
    
    auto request = std::make_shared<mundus_mir_msgs::srv::MissionManager::Request>();
    request->mission_type = mission_type;
    request->force_switch = force;
    
    auto future = client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call mission manager service");
        return false;
    }
    
    auto response = future.get();
    
    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Mission switched successfully to: %s", 
                   response->current_mission.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch mission: %s", 
                    response->message.c_str());
    }
    
    return response->success;
}

void print_usage() {
    std::cout << "Usage: mission_control <mission_type> [--force]" << std::endl;
    std::cout << "  mission_type: pipeline, lawnmower, none" << std::endl;
    std::cout << "  --force: Force immediate switch without safe transition" << std::endl;
}