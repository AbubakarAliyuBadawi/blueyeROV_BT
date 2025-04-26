#!/bin/bash
# Save this as ~/Desktop/blueyeROV_BT/launch_docking.sh
source ~/Desktop/mundus_mir_simulator/install/setup.bash
ros2 launch mundus_mir_docking_controller integrated_docking.launch.py
