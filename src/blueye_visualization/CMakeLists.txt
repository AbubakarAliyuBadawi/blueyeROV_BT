cmake_minimum_required(VERSION 3.5)
project(blueye_visualization)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY blueye_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rov_viz_matplotlib_3d.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rov_viz_matplotlib_3d
)

install(PROGRAMS
  scripts/rov_trajectory_plotter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rov_trajectory_plotter
)

install(PROGRAMS
  scripts/rov_trajectory_plotter_rl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rov_trajectory_plotter_rl
)

install(PROGRAMS
  scripts/2d_rl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME 2d_rl
)

install(PROGRAMS
  scripts/3d_rl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME 3d_rl
)

install(PROGRAMS
  scripts/mission_state_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_state_visualizer
)

install(PROGRAMS
  scripts/system_health_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME system_health_visualizer
)

install(PROGRAMS
  scripts/obstacle_avoidance_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME obstacle_avoidance_visualizer
)

install(PROGRAMS
  scripts/rov_enhanced_2d_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rov_enhanced_2d_trajectory
)

install(PROGRAMS
  scripts/rl_visualization.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_visualization
)

install(PROGRAMS
  scripts/rl_explanation_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rl_explanation_publisher
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
