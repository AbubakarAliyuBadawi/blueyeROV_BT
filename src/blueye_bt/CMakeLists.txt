cmake_minimum_required(VERSION 3.8)
project(blueye_bt)

set(CMAKE_CXX_STANDARD 17) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED) 
find_package(nav_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS} 
)

add_library(waypoint_behaviors SHARED
  src/behaviors/waypoint_behaviors.cpp
  src/behaviors/navigate_to_waypoint.cpp
  src/conditions/battery_condition.cpp
  src/behaviors/station_keeping.cpp
)

add_executable(battery_management src/battery_management.cpp)
add_executable(dock_distance_calc src/dock_distance_calc.cpp)

ament_target_dependencies(waypoint_behaviors
  rclcpp
  behaviortree_cpp 
  mundus_mir_msgs
  std_msgs
)

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

add_executable(blueye_bt src/main.cpp)

ament_target_dependencies(blueye_bt
  rclcpp
  behaviortree_cpp 
  mundus_mir_msgs
  std_msgs
  nav_msgs
)

target_link_libraries(blueye_bt
  waypoint_behaviors
  ${behaviortree_cpp_LIBRARIES}  
)

install(TARGETS
  waypoint_behaviors
  blueye_bt
  battery_management
  dock_distance_calc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()