# blueyeROV_BT (simulation)
A behaviourTree package for the BlueyeROV mission planning for a short video on how to simulation works, here is a youtube video link [Video Link] (https://www.youtube.com/watch?v=vxIXswrNpQY)

![Behavior Tree](./images/bt.png)
*The behaviourTree visualized with Groot2*

## Dependencies

- [BehaviourTree.cpp (v4)](https://github.com/BehaviorTree/BehaviorTree.CPP)

- [Groot2 Visualizer App:](https://www.behaviortree.dev/groot/) If you want to monitor the behaviorTree in real time (Note: since the behaviourTree consist morethan 20 ndoes you need to subscribe to the premium package after the 1 month free trains ends)

This is a separate package that works together with the AURlab Gazebo simulator, to run the simulation you need to clone the simulation package in [mundus_mir_simulator](https://gitlab.com/aurlab/mundus-mir-project/mundus_mir_simulator.git), make sure to switch the branch to "mission_panner" to use the current world the planner uses 

### BehaviorTree.CPP
for more about the documentation of the library 
[BehaviourTree.cpp](https://www.behaviortree.dev/) 

Behaviour tree is implemented with BehaviorTree.CPP is implemented with ROS2 and subscribes to the topics it needs access to and publishes mission states (e.g. waypoint)

## How To Run Mission Planner

### 1st Terminal Simulator

Create a workspace and build both the mission planner and the simulator package

- Run the simulator ros2 "launch mundus_mir_simulator_launch mundus_mir_pipeline_world.launch.py"

### 2nd Terminal Sonar
(you need both the battery and sonar packages to run the mission planner), both packages are included in the source folder, currently a standard sonar based 2d horizontal obstacle avoildance is used for static obstacles (obstacle avoidance is implemented with conventional way) -> would be interesting to get Sam's code in here instead. 

- ros2 launch blueye_bt blueye_bt.launch.py

### 3th Terminal Visualization

- ros2 run blueye_visualization blueye_visualization 

## How To Define Waypoints
Missions are based on XML files in the behaviorTree folder in the bt workspace, you can create your constume missions, but if you need any additional behaviour apart from the behaviours, actions or conditions from the current available once, you need to create a cpp script and register the node in the main file.

The controller is based on the waypoint control Ambjorn implemented in the mundus simulator package, the waypoints uses service call

The overall Mission is defined in Missioncontrol.xml file, but the MissionControl xml file contains subtrees which are included in the behaviorTree folder.

## In Real World
The Blueye Missionplanner SDK is used to create a mission, the BehaviorTree.CPP is used to determine the sequences and calls the according files. rightnow I am working to make this work, and will provide an updated readme.

PyTrees would be other opensource option 
USBL can be used to refer to GPS location in Missionplanner SDK

# blueyeROV_BT (real)
![Behavior Tree](./images/bt_real.png)

- Connect to the blueye wifi

clone the blueye_ws package

# Terminal 1
- cd ~/blueye_ws/
- source install/setup.bash
- ros2 run mundus_mir_docking_controller blueye_commands

# Terminal 2
- cd ~/blueyeROV_BT/
- source install/setup.bash
- ros2 launch blueye_bt_real blueye_bt_real.launch.py

# Terminal 3 (optional)
 To visulize the trajectory of the drone in real time you can run the visulization package

- ros2 launch blueye_visualization_real blueye_visualization_real.launch.py


# Terminal 4 (optional)
- ros2 topic echo /blueye/battery

to simulate the low battery messege to check the reactivite

- ros2 topic pub --once /blueye/battery geometry_msgs/msg/Pose "{position: {x: 10.4, y: 0.0, z: -0.95}, orientation: {x: 3000.0, y: 0.0, z: 0.0, w: 1.0}}"

# Battery Simulation for Blueye ROV Emergency Behavior Testing

The battery condition node subscribes to the `/blueye/battery` topic which uses a `geometry_msgs/msg/Pose` message type as a container for battery information. While this may seem like an unusual message type for battery data, it's used as a convenient container with multiple fields.

### Message Field Mappings

| Pose Field | Battery Information | Description |
|------------|---------------------|-------------|
| `orientation.x` | Runtime to empty | Time remaining in seconds until battery is depleted |
| `position.x` | Charging current | Current flowing into the battery while charging (Amps) |
| `position.z` | Current draw | Current flowing from the battery while discharging (Amps) |

## Battery Level Calculation

The battery percentage is calculated using the following formula:

```cpp
battery_percentage = (runtime_to_empty / full_runtime) * 100.0


## Testing Low Battery Condition

To simulate a low battery condition and trigger the emergency behavior in the behavior tree, you can publish a message with a runtime value that results in a battery percentage below the threshold (default 20%).

### Example Command

```bash
# Simulate approximately 15% battery remaining
ros2 topic pub --once /blueye/battery geometry_msgs/msg/Pose "{position: {x: 10.4, y: 0.0, z: -0.95}, orientation: {x: 3000.0, y: 0.0, z: 0.0, w: 1.0}}"

## Expected Behavior

When the battery level falls below the threshold:

1. The `BatteryLevelCondition` node returns `SUCCESS`.
2. The `ReactiveSequence` in the behavior tree detects this condition.
3. The emergency sequence is triggered, causing the ROV to:
   - Activate auto depth and heading hold.
   - Navigate to the emergency waypoint which are the docking station
   - Dock at the docking station.
