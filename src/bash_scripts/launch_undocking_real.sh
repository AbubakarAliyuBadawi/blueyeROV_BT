#!/bin/bash

# launch_undocking_real.sh
# Script to execute the undocking procedure

echo "Starting undocking procedure..."

# Change to the mission planner scripts directory
cd ~/Desktop/blueyeROV_BT/src/blueye_bt_real/scripts/mission_planner_scripts

# Execute the undocking script
python3 undocking.py --reverse-duration 10 --reverse-power 0.4

# Check if the script executed successfully
if [ $? -eq 0 ]; then
    echo "Undocking procedure completed successfully"
else
    echo "Undocking procedure failed"
    exit 1
fi

echo "Undocking procedure finished"