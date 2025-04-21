#include "gz_battery/gz_battery.h"

// Starts and spins GZ_Battery node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gz_sensors::GZ_Battery>());
    rclcpp::shutdown();
    return 0;
}
