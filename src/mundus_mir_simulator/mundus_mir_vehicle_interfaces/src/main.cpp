#include "mundus_mir_vehicle_interfaces/blueye_simulator_interface.hpp"

// Starts and spins GZ_DVL node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlueyeThrustAllocator>());
    rclcpp::shutdown();
    return 0;
}