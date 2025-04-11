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
# Add Blueye SDK
find_package(blueye_sdk REQUIRED)

# Include directories
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${blueye_sdk_INCLUDE_DIRS}
)

# Add BT nodes library
add_library(blueye_bt_nodes SHARED
  src/actions/navigate_waypoint_action.cpp
  src/conditions/battery_condition.cpp
)

# Add dependencies to the BT nodes library
ament_target_dependencies(blueye_bt_nodes
  rclcpp
  behaviortree_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
  blueye_sdk
)

# Add the main executable
add_executable(blueye_bt_executor src/bt_executor_node.cpp src/main.cpp)

target_link_libraries(blueye_bt_executor
  blueye_bt_nodes
  ${behaviortree_cpp_LIBRARIES}
  ${blueye_sdk_LIBRARIES}
)

ament_target_dependencies(blueye_bt_executor
  rclcpp
  behaviortree_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
  blueye_sdk
)

# Install targets
install(TARGETS
  blueye_bt_nodes
  blueye_bt_executor
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