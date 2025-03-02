cmake_minimum_required(VERSION 3.8)
project(mundus_mir_blueye_joystick_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executables
add_executable(joystick_controller src/mundus_mir_blueye_joystick_cpp.cpp src/main.cpp)

# Include directories
include_directories(include)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Add depdencies
ament_target_dependencies(
  joystick_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  joystick_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
