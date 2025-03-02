cmake_minimum_required(VERSION 3.8)
project(gz_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz_models REQUIRED)
find_package(ros_gz_sim REQUIRED)

# Install launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()