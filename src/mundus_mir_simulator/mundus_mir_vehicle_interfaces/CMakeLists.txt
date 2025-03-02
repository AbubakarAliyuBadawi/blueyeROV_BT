cmake_minimum_required(VERSION 3.8)
project(mundus_mir_vehicle_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)

# Include directories
include_directories(include)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Add executables
add_executable(blueye_simulator_interface src/blueye_simulator_interface.cpp)

# Add dependencies
ament_target_dependencies(blueye_simulator_interface rclcpp geometry_msgs std_msgs Eigen3)

# Install targets
install(TARGETS
  blueye_simulator_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
