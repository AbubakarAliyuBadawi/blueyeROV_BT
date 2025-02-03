cmake_minimum_required(VERSION 3.8)
project(rov_mission_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(mundus_mir_msgs REQUIRED)

# Include directories
include_directories(include)

# Create a list of behavior source files
set(BEHAVIOR_SOURCES
  src/behaviors/waypoint_behaviors.cpp
  src/behaviors/manipulation_behaviors.cpp
  src/behaviors/sensor_behaviors.cpp
)

# Create a list of mission control source files
set(MISSION_CONTROL_SOURCES
  src/mission_control/mission_loader.cpp
  src/mission_control/mission_executor.cpp
  src/mission_control/mission_factory.cpp
)

# Create a list of utility source files
set(UTILS_SOURCES
  src/utils/service_clients.cpp
  src/utils/node_configuration.cpp
)

# Create the main executable
add_executable(rov_mission_bt_node 
  src/main.cpp
  ${BEHAVIOR_SOURCES}
  ${MISSION_CONTROL_SOURCES}
  ${UTILS_SOURCES}
)

# Specify target properties
ament_target_dependencies(rov_mission_bt_node
  rclcpp
  behaviortree_cpp_v3
  mundus_mir_msgs
)

# Install targets
install(TARGETS
  rov_mission_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install mission XML files
install(DIRECTORY
  missions
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()