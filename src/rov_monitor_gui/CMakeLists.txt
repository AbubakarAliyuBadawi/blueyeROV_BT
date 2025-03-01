cmake_minimum_required(VERSION 3.5)
project(rov_monitor_gui)

# Find ament and ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Find Qt5 dependencies
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

# Enable Qt features
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories: add the parent "include" directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${behaviortree_cpp_INCLUDE_DIRS}
)

# Declare the executable
add_executable(rov_monitor_gui
  src/rov_monitor_gui.cpp
)

# Ensure that AUTOMOC processes your header (which contains Q_OBJECT macros)
target_sources(rov_monitor_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rov_monitor_gui/rov_monitor_gui.hpp
)

# Link libraries
target_link_libraries(rov_monitor_gui
  Qt5::Core
  Qt5::Widgets
  ${behaviortree_cpp_LIBRARIES}
)

ament_target_dependencies(rov_monitor_gui
  rclcpp
  nav_msgs
  std_msgs
  mundus_mir_msgs
  behaviortree_cpp
)

# Install the executable
install(TARGETS
  rov_monitor_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
