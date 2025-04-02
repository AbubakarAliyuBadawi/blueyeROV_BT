cmake_minimum_required(VERSION 3.8)
project(blueye_bt_alt)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mundus_mir_msgs_alt REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${mundus_mir_msgs_alt_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

# BT nodes library
add_library(bt_nodes SHARED
  src/behaviors/navigate_to_waypoint.cpp
  src/conditions/battery_condition.cpp
  src/behaviors/station_keeping.cpp
  src/conditions/camera_condition.cpp
  src/conditions/sonar_condition.cpp
  src/conditions/system_watchdog.cpp
)

# Add dependencies to the BT nodes library
ament_target_dependencies(bt_nodes
  rclcpp
  behaviortree_cpp
  mundus_mir_msgs_alt
  nav_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  marine_acoustic_msgs
  std_srvs
)

# Link BT nodes with BehaviorTree.CPP library
target_link_libraries(bt_nodes
  ${behaviortree_cpp_LIBRARIES}
)

# Add remaining executables
add_executable(battery_management src/battery_management.cpp)
add_executable(dock_distance_calc src/dock_distance_calc.cpp)
add_executable(blueye_bt_alt src/main.cpp)

# Link dependencies for each executable
target_link_libraries(battery_management
  ${rclcpp_LIBRARIES}
)
target_link_libraries(dock_distance_calc
  ${rclcpp_LIBRARIES}
)
target_link_libraries(blueye_bt_alt
  bt_nodes
  ${behaviortree_cpp_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

# Add dependencies to executables
ament_target_dependencies(battery_management
  rclcpp
  nav_msgs
  std_msgs
  mundus_mir_msgs_alt
)
ament_target_dependencies(dock_distance_calc
  rclcpp
  nav_msgs
  std_msgs
)
ament_target_dependencies(blueye_bt_alt
  rclcpp
  behaviortree_cpp
  mundus_mir_msgs_alt
  std_msgs
  nav_msgs
  geometry_msgs
  marine_acoustic_msgs
  std_srvs
  sensor_msgs
)

# Install targets
install(TARGETS
  bt_nodes
  blueye_bt_alt
  battery_management
  dock_distance_calc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include
)
install(DIRECTORY
  behavior_trees
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()