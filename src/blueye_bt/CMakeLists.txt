cmake_minimum_required(VERSION 3.8)
project(blueye_bt)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${mundus_mir_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# BT nodes library
add_library(bt_nodes SHARED
  src/behaviors/navigate_to_waypoint.cpp
  src/conditions/battery_condition.cpp
  src/behaviors/station_keeping.cpp
  src/conditions/camera_condition.cpp
  src/conditions/sonar_condition.cpp
  src/conditions/system_watchdog.cpp
  src/behaviors/load_mission_requirements.cpp
  src/conditions/blackboard_condition.cpp 
  src/actions/altitude_control_action.cpp
  src/behaviors/launch_docking_procedure.cpp 
  src/behaviors/wait_node.cpp 
  src/actions/publish_state.cpp
)

# Add dependencies to the BT nodes library
ament_target_dependencies(bt_nodes
  rclcpp
  behaviortree_cpp
  mundus_mir_msgs
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
  ${YAML_CPP_LIBRARIES}
)

# Add remaining executables
add_executable(battery_management src/battery_management.cpp)
add_executable(dock_distance_calc src/dock_distance_calc.cpp)
add_executable(blueye_bt src/main.cpp)
add_executable(waypoint_obstacle_avoidance src/waypoint_obstacle_avoidance.cpp)
add_executable(altitude_controller src/controllers/altitude_controller.cpp)

# Link dependencies for each executable
target_link_libraries(battery_management
  ${rclcpp_LIBRARIES}
)
target_link_libraries(dock_distance_calc
  ${rclcpp_LIBRARIES}
)
target_link_libraries(blueye_bt
  bt_nodes
  ${behaviortree_cpp_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

target_link_libraries(waypoint_obstacle_avoidance
  ${rclcpp_LIBRARIES}
)

target_link_libraries(altitude_controller 
  ${rclcpp_LIBRARIES})

# Add dependencies to executables
ament_target_dependencies(battery_management
  rclcpp
  nav_msgs
  std_msgs
  mundus_mir_msgs
)
ament_target_dependencies(dock_distance_calc
  rclcpp
  nav_msgs
  std_msgs
)
ament_target_dependencies(blueye_bt
  rclcpp
  behaviortree_cpp
  mundus_mir_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  marine_acoustic_msgs
  std_srvs
  sensor_msgs
)

ament_target_dependencies(waypoint_obstacle_avoidance
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  geometry_msgs
  marine_acoustic_msgs
  mundus_mir_msgs
  tf2
)

ament_target_dependencies(altitude_controller
  rclcpp
  nav_msgs
  geometry_msgs
  marine_acoustic_msgs
  std_srvs
)

# Install targets
install(TARGETS
  bt_nodes
  blueye_bt
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

install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(TARGETS
  waypoint_obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  altitude_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()