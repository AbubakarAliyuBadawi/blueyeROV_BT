#include "gz_dvl/gz_dvl.h"

// Starts and spins GZ_DVL node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gz_sensors::GZ_DVL>());
    rclcpp::shutdown();
    return 0;
}