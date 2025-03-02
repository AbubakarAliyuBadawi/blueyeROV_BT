cmake_minimum_required(VERSION 3.8)
project(gz_usbl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(utility_toolbox REQUIRED)
find_package(tf2 REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(gz_usbl src/main.cc src/gz_usbl.cc)

# add dependencies
ament_target_dependencies(gz_usbl rclcpp nav_msgs utility_toolbox tf2)

# install executable
install(TARGETS
    gz_usbl
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
