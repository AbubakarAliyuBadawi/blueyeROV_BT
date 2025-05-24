cmake_minimum_required(VERSION 3.8)
project(blueye_msgs_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionEstimateTel.msg"
  "msg/DepthTel.msg"
  "msg/BatteryTel.msg"
  "msg/DvlVelocityTel.msg"
)


ament_package()