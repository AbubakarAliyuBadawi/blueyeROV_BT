cmake_minimum_required(VERSION 3.8)
project(gz_battery_management)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mundus_mir_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables without .py extension
install(PROGRAMS
  "${PROJECT_NAME}/dock_distance_calc.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_distance_calc
)

install(PROGRAMS
  "${PROJECT_NAME}/battery_management.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME battery_management
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()