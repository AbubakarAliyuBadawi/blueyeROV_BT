cmake_minimum_required(VERSION 3.8)
project(gz_dvl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(utility_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(gz_dvl src/main.cc src/gz_dvl.cc)

# add dependencies
ament_target_dependencies(gz_dvl
  rclcpp
  sensor_msgs
  nav_msgs
  utility_toolbox
  cv_bridge
  geometry_msgs
  marine_acoustic_msgs
)

# install targets
install(TARGETS 
  gz_dvl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
