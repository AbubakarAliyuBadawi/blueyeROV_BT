cmake_minimum_required(VERSION 3.5)
project(gz_robot_monitors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the battery monitor as a library
add_library(battery_monitor_lib
  gz_robot_monitors/battery_monitor.cc
)
ament_target_dependencies(battery_monitor_lib rclcpp mundus_mir_msgs std_msgs)

# Add the geofence monitor as a library
add_library(geofence_monitor_lib
  gz_robot_monitors/geofence_monitor.cc
)
ament_target_dependencies(geofence_monitor_lib rclcpp std_msgs geometry_msgs)

# Add the main executable
add_executable(main
  gz_robot_monitors/main.cc
)
target_link_libraries(main battery_monitor_lib geofence_monitor_lib)
ament_target_dependencies(main rclcpp std_msgs mundus_mir_msgs geometry_msgs)

# Install executables
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export the package
ament_package()
