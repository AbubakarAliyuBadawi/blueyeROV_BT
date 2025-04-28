#!/bin/bash
# Save this as ~/Desktop/blueyeROV_BT/launch_docking_real.sh

source ~/Desktop/thesis/blueye_ws/install/setup.bash
ros2 launch mundus_mir_docking_controller docking.launch.py

#source ~/Desktop/AutonomousDocking/blueye-ros2-interface-autonomous_docking_bjorn_master/install/setup.bash
#ros2 launch real_blueye_bringup real_blueye_ny.launch.py
