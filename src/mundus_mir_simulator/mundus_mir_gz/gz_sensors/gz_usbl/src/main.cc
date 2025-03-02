#include "gz_usbl/gz_usbl.h"

// Starts and spins a GZ_USBL node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gz_sensors::GZ_USBL>());
    rclcpp::shutdown();
    return 0;
}