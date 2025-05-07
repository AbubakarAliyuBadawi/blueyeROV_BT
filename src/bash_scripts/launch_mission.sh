#!/bin/bash
# Shell script to launch the mission planning Python script
# Save as ~/Desktop/blueyeROV_BT/scripts/launch_mission.sh

# Path to the Python script
SCRIPT_PATH=~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/mission_planner_scripts/mission.py

# Execute the script with the provided arguments
python3 $SCRIPT_PATH "$@"

# Return the exit code from the Python script
exit $?
