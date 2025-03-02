cmake_minimum_required(VERSION 3.8)
project(mundus_mir_waypoint_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Inluce include folder
include_directories(include)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Add executables
add_executable(waypoint_controller src/main.cpp src/mundus_mir_waypoint_controller.cpp)

# Add dependencies
ament_target_dependencies(waypoint_controller 
  rclcpp 
  mundus_mir_msgs
  geometry_msgs
  nav_msgs
)

# Install executables
install(TARGETS
  waypoint_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()