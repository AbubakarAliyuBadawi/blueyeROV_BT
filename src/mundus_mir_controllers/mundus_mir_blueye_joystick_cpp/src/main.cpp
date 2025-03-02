#include "mundus_mir_blueye_joystick_cpp/mundus_mir_blueye_joystick_cpp.hpp"

// Starts and spins GZ_DVL node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickController>());
    rclcpp::shutdown();
    return 0;
}