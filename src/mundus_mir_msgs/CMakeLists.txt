cmake_minimum_required(VERSION 3.8)
project(mundus_mir_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Add any additional type support if needed (e.g., FastRTPS or CycloneDDS)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
  "msg/ReturnRecommendation.msg"

  "srv/AddWaypoint.srv"
  "srv/InsertWaypoint.srv"
  "srv/ClearWaypoints.srv"
  "srv/GetWaypoints.srv"
  "srv/RemoveWaypoint.srv"
  "srv/GoToWaypoints.srv"
  "srv/RunWaypointController.srv"
  "srv/GetWaypointStatus.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Ensure type support libraries are available
ament_export_dependencies(rosidl_default_runtime)

ament_package()

