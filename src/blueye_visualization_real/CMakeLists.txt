cmake_minimum_required(VERSION 3.5)
project(blueye_visualization_real)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/blueye_visulization_2d_real.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/blueye_visulization_3d_real.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/blueye_telemetry.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()