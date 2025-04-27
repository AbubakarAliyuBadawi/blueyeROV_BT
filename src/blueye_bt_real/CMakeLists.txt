cmake_minimum_required(VERSION 3.8)
project(blueye_bt_real)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${blueye_sdk_INCLUDE_DIRS}
)

# Add BT nodes library with new name to avoid clash
add_library(blueye_bt_real_nodes SHARED
  src/behaviors/launch_mission_procedure.cpp
  src/behaviors/wait_node_real.cpp 
  src/behaviors/launch_docking_real.cpp
  src/behaviors/launch_depth_mission.cpp
  src/conditions/BatteryLevelCondition.cpp
  src/behaviors/goto_waypoint.cpp
  src/behaviors/goto_waypoint_cc.cpp
)

ament_target_dependencies(blueye_bt_real_nodes
  rclcpp
  behaviortree_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Main executable
add_executable(blueye_bt_real src/main.cpp)

ament_target_dependencies(blueye_bt_real
  rclcpp
  behaviortree_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(blueye_bt_real
  blueye_bt_real_nodes
)

# Install targets
install(TARGETS
  blueye_bt_real_nodes
  blueye_bt_real
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

# Install behavior tree files, launch files, and configs
install(DIRECTORY
  behavior_trees
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
