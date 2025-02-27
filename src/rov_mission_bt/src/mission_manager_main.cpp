#include "rov_mission_bt/managers/mission_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}