cmake_minimum_required(VERSION 3.8)
project(mundus_mir_msgs_alt)

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
  "msg/BatteryStatusAlt.msg"
  "msg/ReturnRecommendationAlt.msg"
  "msg/ActuatorInputAlt.msg"
  "msg/BlueyeCommandAlt.msg"
  "msg/ControlActionsAlt.msg"
  "msg/ControllerStateAlt.msg"
  "msg/DVLAlt.msg"
  "msg/EstimatorStateAlt.msg"
  "msg/ReferenceAlt.msg"
  "msg/ResetPathAlt.msg"
  "msg/SpeedRefAlt.msg"
  
  "srv/AddWaypointAlt.srv"
  "srv/InsertWaypointAlt.srv"
  "srv/ClearWaypointsAlt.srv"
  "srv/GetWaypointsAlt.srv"
  "srv/RemoveWaypointAlt.srv"
  "srv/GoToWaypointsAlt.srv"
  "srv/RunWaypointControllerAlt.srv"
  "srv/GetWaypointStatusAlt.srv"
  "srv/MissionManagerAlt.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Ensure type support libraries are available
ament_export_dependencies(rosidl_default_runtime)
ament_package()