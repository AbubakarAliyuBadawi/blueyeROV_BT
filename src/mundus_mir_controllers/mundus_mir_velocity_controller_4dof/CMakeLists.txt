cmake_minimum_required(VERSION 3.8)
project(velocity_controller_4dof_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pid_siso REQUIRED)

add_executable(velocity_controller_4dof bin/node.cpp src/velocity_controller_4dof.cpp)

ament_target_dependencies(velocity_controller_4dof
  "rclcpp"
  pid_siso
)

target_include_directories(velocity_controller_4dof PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

target_link_libraries(
  velocity_controller_4dof
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
)

target_compile_features(velocity_controller_4dof PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS velocity_controller_4dof
  DESTINATION lib/${PROJECT_NAME})


ament_package()
