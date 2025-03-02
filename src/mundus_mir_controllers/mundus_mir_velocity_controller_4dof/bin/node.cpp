// Spins up ROS2 node

#include "velocity_controller_4dof.hpp"

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController4dof>());
  rclcpp::shutdown();
  return 0;
}