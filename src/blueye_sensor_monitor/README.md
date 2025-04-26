# ROV Sensor Monitoring and Testing Tools

This package contains tools to visualize and test sensor status and emergency behaviors in the underwater ROV system.

## Overview

The tools provided include:

1. **Sensor Visualizer** - Displays real-time status of camera and sonar sensors and logs events
2. **Sensor Failure Simulator** - Allows triggering of sensor failures to test emergency behaviors

These tools are designed to help document when sensors fail and how the behavior tree responds with emergency procedures.

## Setup Instructions

### 1. Add the Python files to your package

Place the following files in your `blueye_bt` package:

```
scripts/
  sensor_visualizer_node.py
  sensor_failure_simulator.py
launch/
  sensor_visualizer.launch.py
  sensor_failure_simulator.launch.py
  sensor_monitoring.launch.py
```

Make sure to set the Python files as executable:

```bash
chmod +x scripts/sensor_visualizer_node.py
chmod +x scripts/sensor_failure_simulator.py
```

### 2. Install Python dependencies

Make sure you have all required Python dependencies:

```bash
pip install matplotlib numpy
```

## Usage

### Basic Usage

To run both the visualizer and failure simulator together:

```bash
ros2 launch blueye_bt sensor_monitoring.launch.py
```

This will:
- Start the sensor visualizer with a real-time timeline display
- Start the sensor failure simulator with command line interface

### Visualizer Only

To run just the visualizer:

```bash
ros2 launch blueye_bt sensor_visualizer.launch.py
```

### Failure Simulator Only

To run just the failure simulator:

```bash
ros2 launch blueye_bt sensor_failure_simulator.launch.py
```

You can customize the failure durations:

```bash
ros2 launch blueye_bt sensor_failure_simulator.launch.py camera_failure_duration:=60.0 sonar_failure_duration:=45.0
```

To disable auto-recovery (failures stay active until manually recovered):

```bash
ros2 launch blueye_bt sensor_failure_simulator.launch.py auto_recover:=false
```

### Triggering Failures via Command Line

When the failure simulator is running, you'll see a command line interface:

```
Sensor Failure Simulator CLI
----------------------------
Commands:
  c - Trigger camera failure
  s - Trigger sonar failure
  b - Trigger both failures
  r - Recover all sensors
  q - Quit
```

Simply type the corresponding letter and press Enter to trigger failures.

### Triggering Failures via ROS 2 Service

You can also trigger failures programmatically using ROS 2 services:

```bash
# Trigger camera failure
ros2 service call /failure_simulator/trigger_camera_failure std_srvs/srv/SetBool "{data: true}"

# Trigger sonar failure
ros2 service call /failure_simulator/trigger_sonar_failure std_srvs/srv/SetBool "{data: true}"
```

## Data Collection

The sensor visualizer automatically collects data for your report:

- **Timeline Image**: A PNG image showing the sensor status timeline is saved to `/tmp/sensor_timeline_*.png`
- **Event Log**: A CSV file with all sensor events is saved to `/tmp/sensor_log_*.csv`
- **Summary Report**: A text summary of incidents is saved to `/tmp/sensor_report_*.txt`

These files are automatically saved when you stop the visualizer (Ctrl+C).

## Testing Emergency Behaviors

To test and document emergency behaviors for your report:

1. Launch your main ROV behavior tree
2. Launch the sensor monitoring tools
3. Wait for the mission to reach a steady state
4. Trigger a sensor failure (e.g., type 's' to fail the sonar)
5. Observe the behavior tree response in the visualizer
6. Document the sequence of events from the visualizer's output files

### Example Test Scenarios

1. **Sonar Failure During Transit**:
   - Start the mission
   - Wait until Transit_1 phase
   - Trigger sonar failure
   - Observe emergency return behavior

2. **Camera Failure During Inspection**:
   - Start the mission
   - Wait until Pipeline Inspection phase
   - Trigger camera failure 
   - Observe emergency return behavior

3. **Recovery Before Emergency**:
   - Start the mission
   - Trigger short failure (auto-recovers)
   - Verify mission continues normally

## Report Generation

After running tests, you can use the generated files for your report:

1. Include the timeline images to show sensor status during the mission
2. Use the event logs to create tables of failure incidents
3. Quote from the summary reports for quantitative analysis

The CSV files can be imported into spreadsheet software for further analysis and chart creation.



python3 analyze_sensor_log.py /tmp/sensor_log_20250418_040130.csv --output-dir /home/badawi/Desktop/blueyeROV_BT/src/