# blueyeROV_BT
A behaviourTree package for the BlueyeROV mission planning

## Dependencies
BehaviorTree.CPP: https://github.com/BehaviorTree/BehaviorTree.CPP (not free)

### BehaviorTree.CPP
is implemented with ROS2 and subscribes to the topics it needs access to and publishes mission states (e.g. waypoint)

## How To Run Mission Planner
You need to run the simulator seperately in a terminal first, use:
ros2 launch mundus_mir_simulator_launch mundus_mir_pipeline_world.launch.py

Behaviour tree is implemented with BehaviorTree.CPP

Sonar is essential for mission, seperate package taken from blueye ws, currently obstacle avoidance is implemented with conventional way -> would be interesting to get Sam's code in here instead. 

In another terminal run sonar with: 
ros2 run gz_sonar gz_sonar

To launch mission planner package (including battery package)
ros2 launch blueye_bt blueye_bt.launch.py

In another Terminal run (from Mundusmir simulator package):
ros2 run blueye_visualization blueye_visualization 

## How To Define Waypoints
Simulation is based on Ambjorn's waypoint controller
waypoints are called for using Ambjorn's service call
Mission is defined in Missioncontrol.xml file, that is created using BehaviorTree.CPP
The lower level movements are defined in sub-xml files

## In Real World
The Blueye Missionplanner SDK is used to create a mission, the BehaviorTree.CPP is used to determine the sequences and calls the according files.
PyTrees would be other opensource option 
USBL can be used to refer to GPS location in Missionplanner SDK
