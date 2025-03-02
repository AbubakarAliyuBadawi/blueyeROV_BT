# mundus_mir_simulator_launch


## Introduction

This repository contains all the world launch files of the Mundus Mir Simulator. 


## Structure

There are two folders in this repository, *launch* and *configuration*. The *launch* directory contains the simulation launch files, while the *config* folder contains the corresponding configuration files.

<pre> ``` 
mundur_mir_simulator_launch/ 
├── README.md 
├── CMakeLists.txt  
├── package.xml
├── launch/ 
│   └── launch_file_1.launch.py
│   └── launch_file_2.launch.py
│   └── ...
├── config/
    └── launch_file_1/
        └── bridge.yaml
        └── controller.yaml
        └── joystick.yaml
        └── navigation_filter.yaml
        └── robot.yaml
    └── launch_file_2/
        └── ...
    └── ...
``` </pre>

- **bridge.yaml:** Parameter file for the ros2-gz bridge.
- **controller.yaml:** Parameters for the controllers used in a launch file.
- **joystick.yaml:** Parameters for mapping between joystick and ros2 joystick message.
- **navigation_filter.yaml:** Parameters of navigation filter used in launch file.
- **robot.yaml:** Parameters of the robot, sensors etc.  





## Joystick Configuration

For some reason the mapping between buttons and the ros2 joystick message seems to differ between systems. The default values may not correspond to the way your joystick is mapped on your system. In order to make sure the mapping is correct you can change the parameters in the joystick.yaml file of your project. 


These are the default values:
```yaml
joy_topic: '/joy'
x_axis_left_stick: 0
y_axis_left_stick: 1
x_axis_right_stick: 3
y_axis_right_stick: 4
cross_button: 0
circle_button: 1 
square_button: 2 
triangle_button: 3 
```

In order to find the mapping of your joystick connect it to your computer and run the *joy_node* and subscribe to the joy topic.

```
ros2 run joy joy_node
ros2 topic echo /joy
```

Move the sticks and click the buttons and see if they correlate with the default values, if they don't you can go into the yaml file and make changes accordingly. 

