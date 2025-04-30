- Connect to the blueye wifi

# Terminal 1
cd ~/Desktop/thesis/blueye_ws/
source install/setup.bash
ros2 run mundus_mir_docking_controller blueye_commands

# Terminal 2
cd ~/Desktop/blueyeROV_BT/
source install/setup.bash
ros2 launch blueye_bt_real blueye_bt_real.launch.py


# Terminal 3 (optional)
ros2 topic echo /blueye/battery

to simulate the low battery messege to check the reactivite

ros2 topic pub --once /blueye/battery geometry_msgs/msg/Pose "{position: {x: 10.4, y: 0.0, z: -0.95}, orientation: {x: 3000.0, y: 0.0, z: 0.0, w: 1.0}}"