cmake_minimum_required(VERSION 3.8)
project(gz_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mundus_mir_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(gz_battery_node src/main.cpp src/gz_battery.cpp)

# Link dependencies to the executable
ament_target_dependencies(gz_battery_node
  rclcpp
  std_msgs
  mundus_mir_msgs
)

# Install the executable target
install(TARGETS 
  gz_battery_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Define ament_package last
ament_package()
