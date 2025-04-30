# Blueye Drone Position Reset and Autonomous Mission

This repository contains scripts for executing autonomous drone missions with the Blueye underwater drone using its SDK. The key feature is the ability to programmatically set the drone's starting position using GPS coordinates without needing the Blueye app.

## Problem Solved

The standard Blueye SDK doesn't provide a direct way to set the starting position of the drone, which is required for autopilot missions. Without setting a position, the drone will show the error:

> The drone position is unknown, unable to start autopilot mission

This usually requires manually setting the position through the Blueye app before running any autonomous mission.

## Solution

This implementation extends the Blueye SDK to add position reset functionality, allowing missions to be executed fully programmatically.

## Files

### 1. `reset_position.py`

This utility module extends the Blueye SDK's `CtrlClient` class with a `reset_position` method that allows you to programmatically reset the drone's position to a specified GPS coordinate.

The module implements the same functionality as the "Reset to POI" function in the Blueye app by creating and sending a `ResetPositionCtrl` message with appropriate settings.

### 2. `pool_test_with_position_reset.py`

This is the main mission script that:

1. Connects to the Blueye drone
2. Extends the drone's control client with the reset position functionality
3. Resets the drone's position to specified starting coordinates
4. Creates a 5-waypoint mission for pipeline inspection and docking:
   - 4 pipeline inspection waypoints at 0.5m depth
   - 1 docking station waypoint at 2.3m depth
5. Executes and monitors the mission

## How to Use

1. Ensure both files are in the same directory.

2. Install the Blueye SDK if you haven't already:
   ```
   pip install blueye.sdk
   ```

3. Run the mission with optional parameters:
   ```
   python pool_test_with_position_reset.py [options]
   ```

### Command Line Options

- `--drone-ip IP_ADDRESS`: IP address of the Blueye drone (default: 192.168.1.101)
- `--timeout SECONDS`: Connection timeout for the drone in seconds (default: 30)
- `--start-lat LATITUDE`: Starting latitude position to set (default: 63.441475)
- `--start-lon LONGITUDE`: Starting longitude position to set (default: 10.348348)
- `--start-heading DEGREES`: Starting heading in degrees (default: 0.0)

## Technical Implementation Details

The implementation accesses the Blueye drone's control message queue directly and sends a properly formatted `ResetPositionCtrl` message with appropriate settings:

```python
reset_settings = {
    "heading_source_during_reset": bp.HeadingSource.HEADING_SOURCE_MANUAL_INPUT,
    "manual_heading": heading,
    "reset_coordinate_source": bp.ResetCoordinateSource.RESET_COORDINATE_SOURCE_MANUAL,
    "reset_coordinate": {
        "latitude": lat,
        "longitude": lon
    },
    "heading_mode": bp.HeadingMode.HEADING_MODE_MAGNETIC_COMPASS
}

msg = bp.ResetPositionCtrl(settings=reset_settings)
drone.connection._ctrl_client._messages_to_send.put(msg)
```

This approach modifies the drone's position estimate without changing the SDK's core code.

## Waypoints

The mission follows these waypoints:

```
Pipeline Point 1: (63.441475, 10.348348) at 0.5m depth
Pipeline Point 2: (63.441478, 10.348247) at 0.5m depth
Pipeline Point 3: (63.441422, 10.348239) at 0.5m depth
Pipeline Point 4: (63.441418, 10.348347) at 0.5m depth
Docking Station: (63.441441, 10.348331) at 2.3m depth
```

## Limitations and Notes

- This is a temporary solution until the functionality is officially added to the Blueye SDK
- The implementation depends on internal SDK structures that might change in future updates
- The drone requires stable position data for accurate navigation
- For optimal performance, perform a proper compass calibration before starting missions

## License

MIT License


## TODO

1 - test dept with this script (at the pear)

cd /home/badawi/Desktop/thesis/BlueyeROV_Autonomous_Docking/src/ocean_test

python simple_depth_mission.py --depth 1.5 --duration 180 --start-lat 63.441475 --start-lon 10.348348

2 - test retry gabriels docking

cd /home/badawi/Desktop/thesis/blueye_ws

ros2 run joy joy_node

ros2 run blueye_joystick_cpp joystick_controller

B heading hold
Y dept hold
X docking

ros2 launch mundus_mir_docking_controller docking.launch.py

3 - test bjorn docking 

cd Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/

export ROS_DOMAIN_ID=10

ros2 launch real_blueye_bringup real_blueye_ny.launch.py

error bjorn's code (FIX THE REAL LIFE POSE ESTIMATION NODE)

badawi@badawi:~/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master$ ros2 launch real_blueye_bringup real_blueye_ny.launch.py
[INFO] [launch]: All log files can be found below /home/badawi/.ros/log/2025-04-26-14-55-42-832766-badawi-12536
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [tcp_server_node-1]: process started with pid [12539]
[INFO] [ekf_node-2]: process started with pid [12541]
[INFO] [robot_state_publisher-3]: process started with pid [12543]
[INFO] [pose_estimation_aruco_real_life_node-4]: process started with pid [12545]
[INFO] [blueye_commands-5]: process started with pid [12547]
[INFO] [blueye_camera_node-6]: process started with pid [12549]
[robot_state_publisher-3] [INFO] [1745672143.206525510] [robot_state_publisher]: got segment base_link
[robot_state_publisher-3] [INFO] [1745672143.206628003] [robot_state_publisher]: got segment camera_link
[robot_state_publisher-3] [INFO] [1745672143.206635875] [robot_state_publisher]: got segment imu_link
[tcp_server_node-1] [INFO] [1745672143.570865825] [tcp_server]: Connected to DVL at 192.168.1.99:16171
[blueye_camera_node-6] [ WARN:0@1.183] global cap_gstreamer.cpp:1697 open OpenCV | GStreamer warning: unable to query duration of stream
[blueye_camera_node-6] [ WARN:0@1.183] global cap_gstreamer.cpp:1728 open OpenCV | GStreamer warning: Cannot query video position: status=1, value=0, duration=-1
[pose_estimation_aruco_real_life_node-4] [[-7.5183868e-02  7.5925946e-02  1.2173653e-03]
[pose_estimation_aruco_real_life_node-4]  [ 7.5785160e-02  7.5420976e-02 -1.5783310e-04]
[pose_estimation_aruco_real_life_node-4]  [ 7.5289011e-02 -7.6133013e-02 -1.2288094e-03]
[pose_estimation_aruco_real_life_node-4]  [-7.5773001e-02 -7.5348020e-02  5.6266785e-05]]
[pose_estimation_aruco_real_life_node-4] [[-0.4394499  -0.4262911  -1.0100079 ]
[pose_estimation_aruco_real_life_node-4]  [-0.28821886 -0.42451    -1.0113297 ]
[pose_estimation_aruco_real_life_node-4]  [-0.28639793 -0.57547104 -1.0123308 ]
[pose_estimation_aruco_real_life_node-4]  [-0.43752885 -0.577605   -1.0105858 ]]
[pose_estimation_aruco_real_life_node-4] [[ 0.32031608 -0.4252311  -1.0182908 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.47201514 -0.42649603 -1.0194368 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.4706421  -0.577767   -1.0188258 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.31939912 -0.576532   -1.0186768 ]]
[pose_estimation_aruco_real_life_node-4] [[-0.11212993 -0.8735801  -0.00492191]
[pose_estimation_aruco_real_life_node-4]  [ 0.10947609 -0.87413704 -0.00570583]
[pose_estimation_aruco_real_life_node-4]  [ 0.10861206 -1.096304   -0.0064826 ]
[pose_estimation_aruco_real_life_node-4]  [-0.11318994 -1.0957501  -0.00574589]]
[pose_estimation_aruco_real_life_node-4] [[-0.06405997 -0.422253   -1.0153787 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.08698511 -0.42243004 -1.0164168 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.08666921 -0.57359505 -1.0167847 ]
[pose_estimation_aruco_real_life_node-4]  [-0.0643158  -0.57329607 -1.0155938 ]]
[pose_estimation_aruco_real_life_node-4] [[ 0.5642941  -0.7250891  -0.01084375]
[pose_estimation_aruco_real_life_node-4]  [ 0.6704891  -0.7251301  -0.01145983]
[pose_estimation_aruco_real_life_node-4]  [ 0.6707461  -0.831393   -0.01049471]
[pose_estimation_aruco_real_life_node-4]  [ 0.56446123 -0.8312861  -0.01019478]]
[pose_estimation_aruco_real_life_node-4] [[-0.04317594 -0.67567706 -1.0171077 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.06334805 -0.67488706 -1.0167148 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.06392121 -0.78148806 -1.0164506 ]
[pose_estimation_aruco_real_life_node-4]  [-0.04271388 -0.78178906 -1.0156507 ]]
[pose_estimation_aruco_real_life_node-4] [[-0.2796259  -0.02939606 -0.00517464]
[pose_estimation_aruco_real_life_node-4]  [-0.20905387 -0.02841306 -0.00543594]
[pose_estimation_aruco_real_life_node-4]  [-0.20800889 -0.098948   -0.00589085]
[pose_estimation_aruco_real_life_node-4]  [-0.2786429  -0.10000002 -0.00579786]]
[pose_estimation_aruco_real_life_node-4] [[-0.51427794 -0.16110504 -0.00596189]
[pose_estimation_aruco_real_life_node-4]  [-0.44429994 -0.16164303 -0.00389957]
[pose_estimation_aruco_real_life_node-4]  [-0.44452584 -0.231982   -0.00341988]
[pose_estimation_aruco_real_life_node-4]  [-0.5144539  -0.23157108 -0.00625753]]
[pose_estimation_aruco_real_life_node-4] [[ 1.199578   -0.8561671  -0.02394295]
[pose_estimation_aruco_real_life_node-4]  [ 1.2700052  -0.8574351  -0.02506685]
[pose_estimation_aruco_real_life_node-4]  [ 1.2695141  -0.92755103 -0.02505493]
[pose_estimation_aruco_real_life_node-4]  [ 1.1989641  -0.926728   -0.02421188]]
[pose_estimation_aruco_real_life_node-4] [[ 0.271981   -0.02639008 -0.01071692]
[pose_estimation_aruco_real_life_node-4]  [ 0.34269214 -0.02691603 -0.01174593]
[pose_estimation_aruco_real_life_node-4]  [ 0.3424902  -0.09726501 -0.01222372]
[pose_estimation_aruco_real_life_node-4]  [ 0.271631   -0.09704101 -0.01177263]]
[pose_estimation_aruco_real_life_node-4] [[ 1.3132491  -0.02161205 -1.0307868 ]
[pose_estimation_aruco_real_life_node-4]  [ 1.2423372  -0.02140105 -1.0296898 ]
[pose_estimation_aruco_real_life_node-4]  [ 1.2420061  -0.09203911 -1.0303268 ]
[pose_estimation_aruco_real_life_node-4]  [ 1.3124521  -0.09197307 -1.0313258 ]]
[pose_estimation_aruco_real_life_node-4] [[ 1.2201481  -0.02911305 -0.02044153]
[pose_estimation_aruco_real_life_node-4]  [ 1.2903731  -0.02857304 -0.02092457]
[pose_estimation_aruco_real_life_node-4]  [ 1.29075    -0.09970605 -0.0218029 ]
[pose_estimation_aruco_real_life_node-4]  [ 1.220612   -0.09972608 -0.02128172]]
[pose_estimation_aruco_real_life_node-4] [[-0.51366484 -0.7511411  -0.00871277]
[pose_estimation_aruco_real_life_node-4]  [-0.44343984 -0.75088406 -0.00883389]
[pose_estimation_aruco_real_life_node-4]  [-0.4439119  -0.82140505 -0.00887156]
[pose_estimation_aruco_real_life_node-4]  [-0.51440394 -0.8209981  -0.00875998]]
[pose_estimation_aruco_real_life_node-4] [[-0.06392694 -0.34224904 -1.0148907 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.00665402 -0.3423381  -1.0155888 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.0062542  -0.4128821  -1.0157259 ]
[pose_estimation_aruco_real_life_node-4]  [-0.064255   -0.4126501  -1.0147898 ]]
[pose_estimation_aruco_real_life_node-4] [[ 0.01680923 -0.34244204 -1.0154028 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.08729911 -0.34242105 -1.0159938 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.08686018 -0.41304708 -1.0160737 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.01657414 -0.41276503 -1.0162048 ]]
[pose_estimation_aruco_real_life_node-4] [[ 0.5656612  -0.11944604 -0.00724983]
[pose_estimation_aruco_real_life_node-4]  [ 0.7876582  -0.1211921  -0.00774384]
[pose_estimation_aruco_real_life_node-4]  [ 0.78621006 -0.3434651  -0.00952387]
[pose_estimation_aruco_real_life_node-4]  [ 0.564481   -0.3418491  -0.00917768]]
[pose_estimation_aruco_real_life_node-4] [[ 1.4402902  -0.12312007 -0.4200077 ]
[pose_estimation_aruco_real_life_node-4]  [ 1.43734    -0.12326503 -0.64142656]
[pose_estimation_aruco_real_life_node-4]  [ 1.4375532  -0.34568405 -0.64136887]
[pose_estimation_aruco_real_life_node-4]  [ 1.4401102  -0.34523308 -0.42013788]]
[pose_estimation_aruco_real_life_node-4] [[-0.5463909  -0.1119101  -0.6219306 ]
[pose_estimation_aruco_real_life_node-4]  [-0.5422379  -0.11489308 -0.40121365]
[pose_estimation_aruco_real_life_node-4]  [-0.5415909  -0.33635008 -0.4033947 ]
[pose_estimation_aruco_real_life_node-4]  [-0.5445689  -0.33385205 -0.6247306 ]]
[pose_estimation_aruco_real_life_node-4] [[ 0.62229514 -0.11684501 -1.0342119 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.40047407 -0.11817205 -1.0329688 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.40192604 -0.3401841  -1.0334098 ]
[pose_estimation_aruco_real_life_node-4]  [ 0.62334514 -0.33892703 -1.0346627 ]]
[pose_estimation_aruco_real_life_node-4] [[-0.22193193 -0.11289001 -1.0272188 ]
[pose_estimation_aruco_real_life_node-4]  [-0.44330585 -0.11320806 -1.0236187 ]
[pose_estimation_aruco_real_life_node-4]  [-0.44291592 -0.3353001  -1.0243247 ]
[pose_estimation_aruco_real_life_node-4]  [-0.22116983 -0.33498406 -1.0282748 ]]
[pose_estimation_aruco_real_life_node-4] Traceback (most recent call last):
[pose_estimation_aruco_real_life_node-4]   File "/home/badawi/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/image_processing/lib/image_processing/pose_estimation_aruco_real_life_node", line 33, in <module>
[pose_estimation_aruco_real_life_node-4]     sys.exit(load_entry_point('image-processing==0.0.0', 'console_scripts', 'pose_estimation_aruco_real_life_node')())
[pose_estimation_aruco_real_life_node-4]   File "/home/badawi/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/image_processing/lib/python3.10/site-packages/image_processing/pose_estimation_aruco_real_life.py", line 602, in main
[pose_estimation_aruco_real_life_node-4]     rclpy.spin(pose_estimation_aruco_node)
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 222, in spin
[pose_estimation_aruco_real_life_node-4]     executor.spin_once()
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 739, in spin_once
[pose_estimation_aruco_real_life_node-4]     self._spin_once_impl(timeout_sec)
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 736, in _spin_once_impl
[pose_estimation_aruco_real_life_node-4]     raise handler.exception()
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 239, in __call__
[pose_estimation_aruco_real_life_node-4]     self._handler.send(None)
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 437, in handler
[pose_estimation_aruco_real_life_node-4]     await call_coroutine(entity, arg)
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 362, in _execute_subscription
[pose_estimation_aruco_real_life_node-4]     await await_or_execute(sub.callback, msg)
[pose_estimation_aruco_real_life_node-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
[pose_estimation_aruco_real_life_node-4]     return callback(*args)
[pose_estimation_aruco_real_life_node-4]   File "/home/badawi/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/image_processing/lib/python3.10/site-packages/image_processing/pose_estimation_aruco_real_life.py", line 277, in image_callback
[pose_estimation_aruco_real_life_node-4]     output_image = self.pose_estimation(image, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
[pose_estimation_aruco_real_life_node-4]   File "/home/badawi/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/image_processing/lib/python3.10/site-packages/image_processing/pose_estimation_aruco_real_life.py", line 397, in pose_estimation
[pose_estimation_aruco_real_life_node-4]     cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
[pose_estimation_aruco_real_life_node-4] AttributeError: module 'cv2.aruco' has no attribute 'Dictionary_get'. Did you mean: 'Dictionary'?
[blueye_commands-5] [INFO] [1745672145.050158964] [blueye_commands]: Connected to Blueye drone.
[ERROR] [pose_estimation_aruco_real_life_node-4]: process has died [pid 12545, exit code 1, cmd '/home/badawi/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/image_processing/lib/image_processing/pose_estimation_aruco_real_life_node --ros-args -r __node:=pose_estimation_aruco_real_life'].


badawi@badawi:~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts$ python3 goto_waypoint.py 
Usage: python goto_waypoint.py <waypoint_lat> <waypoint_lon> <depth> <drone_ip>
badawi@badawi:~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts$ python3 goto_waypoint_cc.py 
Usage: python goto_waypoint.py <waypoint_lat> <waypoint_lon> <depth> <drone_ip> <initialize_connection>
badawi@badawi:~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts$ 
