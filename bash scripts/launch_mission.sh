#!/bin/bash

SCRIPT_PATH=/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/mission_planner_scripts/mission.py

# Execute the script with the provided arguments
python3 $SCRIPT_PATH "$@"

# Return the exit code from the Python script
exit $?