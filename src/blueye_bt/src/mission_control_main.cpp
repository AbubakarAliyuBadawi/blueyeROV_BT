#include "blueye_bt/managers/mission_control.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        print_usage();
        return 1;
    }
    
    std::string mission_type = argv[1];
    bool force = false;
    
    if (argc >= 3 && std::string(argv[2]) == "--force") {
        force = true;
    }
    
    auto client = std::make_shared<MissionControlClient>();
    bool success = client->switch_mission(mission_type, force);
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}