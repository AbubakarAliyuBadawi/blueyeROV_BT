cmake_minimum_required(VERSION 3.8)
project(rov_mission_bt)

set(CMAKE_CXX_STANDARD 17)  # Changed to C++17 for v4
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)  # Changed from v3
find_package(nav_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ZMQ is included in behaviortree_cpp
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS} 
)

# Waypoint behaviors library
add_library(waypoint_behaviors SHARED
  src/behaviors/waypoint_behaviors.cpp
  src/behaviors/navigate_to_waypoint.cpp
  src/conditions/battery_condition.cpp
  src/behaviors/station_keeping.cpp
)

ament_target_dependencies(waypoint_behaviors
  rclcpp
  behaviortree_cpp 
  mundus_mir_msgs
  std_msgs
)

# Main executable
add_executable(rov_mission_bt src/main.cpp)

ament_target_dependencies(rov_mission_bt
  rclcpp
  behaviortree_cpp 
  mundus_mir_msgs
  std_msgs
  nav_msgs
)

target_link_libraries(rov_mission_bt
  waypoint_behaviors
  ${behaviortree_cpp_LIBRARIES}  # Added explicit linking
)

# Install targets
install(TARGETS
  waypoint_behaviors
  rov_mission_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories if they exist
install(DIRECTORY
  behavior_trees
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()