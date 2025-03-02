# velocity_controller_4dof_ros2

This repository includes a ROS2 package for a velocity controller that's built on the generalized controller [pid_siso_cpp](https://gitlab.com/aurlab/rov-control-system/controllers/pid_siso_cpp).

## Dependencies

    - eigen3/Eigen/Dense
    - pid_siso_cpp
    - velocity_controller_4dof_ros2_interfaces


## How to build

Clone repo with submodules:

SSH:


    git clone --recurse-submodules git@gitlab.com:aurlab/rov-control-system/controllers/velocity_controller_4dof_ros2.git

and for HTTPS:

    git clone --recurse-submodules https://gitlab.com/aurlab/rov-control-system/controllers/velocity_controller_4dof_ros2.git

build with `colcon`:

    colcon build --packages-select velocity_controller_4dof_ros2



## Python launch description with parameters
Parameters annotated `rt` can be changed online. The others has to be set prior to node spin.



    # Velocity controller
    velocity_controller = Node(
            package='velocity_controller_4dof_ros2',
            executable='velocity_controller_4dof',
            name='velocity_controller',
            parameters=[
                {"qos_buffer_size": 10},
                {"topic_subscriber_odometry": "/sub_odom_odometry"},
                {"topic_subscriber_desired": "/sub_odom_desired"},
                {"topic_publisher": "/wrench"},
                {"controller_frequency_hz": 100},
                {"rt_surge_kp": 0.1},
                {"rt_surge_ki": 0.0},
                {"rt_surge_kd": 0.0},
                {"rt_surge_feedforward": 0.0},
                {"rt_surge_satUpper": 1.0},
                {"rt_surge_satLower": -1.0},
                {"rt_sway_kp": 0.1},
                {"rt_sway_ki": 0.0},
                {"rt_sway_kd": 0.0},
                {"rt_sway_feedforward": 0.0},
                {"rt_sway_satUpper": 1.0},
                {"rt_sway_satLower": -1.0},
                {"rt_depth_kp": 0.1},
                {"rt_depth_ki": 0.0},
                {"rt_depth_kd": 0.0},
                {"rt_depth_feedforward": 0.0},
                {"rt_depth_satUpper": 1.0},
                {"rt_depth_satLower": -1.0},
                {"rt_yaw_kp": 0.2},
                {"rt_yaw_ki": 0.0},
                {"rt_yaw_kd": 0.0},
                {"rt_yaw_feedforward": 0.0},
                {"rt_yaw_satUpper": 1.0},
                {"rt_yaw_satLower": -1.0}
            ]
            )



## Service calls
The node has one available service calls for resetting the integral terms to zero, which can called from CLI:

    ros2 service call /velocity_controller_4dof/set_integral_reset controller_4dof_ros2_interfaces/srv/IntegralReset "{surge: True, sway: True, depth: True, yawrate: False}"

