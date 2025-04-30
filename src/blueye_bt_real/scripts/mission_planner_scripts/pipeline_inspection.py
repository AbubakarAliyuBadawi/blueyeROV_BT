#!/usr/bin/env python3
"""
Pool Test Mission Script for Blueye Drone with Position Reset

This script executes an autonomous pipeline inspection and docking mission with 5 waypoints:
1-4. Navigate along pipeline waypoints at 0.5 meters depth
5. Navigate to the docking station and descend to 2.3 meters for docking

Pipeline waypoints:
- Point 1: (63.441475, 10.348348)
- Point 2: (63.441478, 10.348247)
- Point 3: (63.441422, 10.348239)
- Point 4: (63.441418, 10.348347)
- Docking: (63.441441, 10.348331)

Usage:
  python pool_test_with_position_reset.py [--drone-ip IP_ADDRESS] [--timeout SECONDS]
"""

import argparse
import sys
import time
import logging
from pathlib import Path
from datetime import datetime

# Import required Blueye SDK components
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition

# Import our position reset extension
from reset_drone_position import extend_ctrl_client


def setup_logging(log_file="log/pipeline_inspection.log", log_level="INFO"):
    """Set up logging configuration."""
    # Create log directory if needed
    log_path = Path(log_file)
    log_path.parent.mkdir(exist_ok=True, parents=True)
    
    # Set up logging level
    level = getattr(logging, log_level.upper())
    
    # Configure logging
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    return logging.getLogger("pool_test")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Run a pipeline inspection and docking mission with position reset')
    
    # Drone settings
    parser.add_argument('--drone-ip', type=str, default="192.168.1.101",
                        help='IP address of the Blueye drone')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Connection timeout for the drone in seconds')
    
    # Starting position settings
    parser.add_argument('--start-lat', type=float, default=63.441475,
                        help='Starting latitude position to set')
    parser.add_argument('--start-lon', type=float, default=10.348348,
                        help='Starting longitude position to set')
    parser.add_argument('--start-heading', type=float, default=0.0,
                        help='Starting heading in degrees')
    
    # Parse arguments
    return parser.parse_args()


def connect_to_drone(ip, timeout, logger):
    """Connect to the drone."""
    logger.info(f"Connecting to drone at {ip}")
    try:
        drone = Drone(
            ip=ip,
            auto_connect=True,
            timeout=timeout,
        )
        
        logger.info(f"Connected to drone: {drone.serial_number}")
        logger.info(f"Drone software version: {drone.software_version}")
        
        if not drone.in_control:
            logger.info("Taking control of drone...")
            drone.take_control()
            logger.info("Control of drone acquired")
        
        # Extend the drone with the reset_position functionality
        drone = extend_ctrl_client(drone)
        logger.info("Drone control client extended with reset_position functionality")
        
        return drone
        
    except Exception as e:
        logger.error(f"Failed to connect to drone: {str(e)}")
        return None


def reset_drone_position(drone, lat, lon, heading, logger):
    """Reset the drone's position."""
    logger.info(f"Resetting drone position to coordinates: {lat}, {lon} with heading: {heading}°")
    try:
        # Call our added reset_position method
        drone._ctrl_client.reset_position(lat, lon, heading)
        logger.info("Position reset command sent successfully")
        
        # Give some time for the position to be updated
        time.sleep(10)
        return True
    except Exception as e:
        logger.error(f"Failed to reset drone position: {str(e)}")
        return False


def create_mission(logger):
    """Create a mission with pipeline survey waypoints and docking station."""
    logger.info("Creating pipeline survey")
    
    ## TOO LONG TO COMPLETE
    # pipeline_waypoints = [
    # {"lat": 63.4406620, "lon": 10.3488502, "name": "Pipeline Point 1"},
    # {"lat": 63.4406063, "lon": 10.3488254, "name": "Pipeline Point 2"},
    # {"lat": 63.4406029, "lon": 10.3489612, "name": "Pipeline Point 3"},
    # {"lat": 63.4406575, "lon": 10.3489893, "name": "Pipeline Point 4"},
    # ]
    # docking_station = {
    # "lat": 63.4406991,
    # "lon": 10.3489964,
    # "name": "Docking Station",
    # "depth": 2.3
    # }
    
    pipeline_waypoints = [
    {"lat": 63.4406850, "lon": 10.3489600, "name": "Pipeline Point 1"},
    {"lat": 63.4406650, "lon": 10.3489300, "name": "Pipeline Point 2"},
    {"lat": 63.4406700, "lon": 10.3489800, "name": "Pipeline Point 3"},
    {"lat": 63.4406850, "lon": 10.3489850, "name": "Pipeline Point 4"},
    ]
    docking_station = {
    "lat": 63.4406991,
    "lon": 10.3489964,
    "name": "Docking Station",
    "depth": 2.3
    }
    
    # Create the instructions for the mission
    instructions = []
    instruction_id = 1
    
    # Step 1: Configure auto-depth mode
    control_mode = bp.Instruction(
        id=instruction_id,
        control_mode_command=bp.ControlModeCommand(
            control_mode_vertical=bp.ControlModeVertical.CONTROL_MODE_VERTICAL_AUTO_DEPTH,
            control_mode_horizontal=bp.ControlModeHorizontal.CONTROL_MODE_HORIZONTAL_AUTO_HEADING
        ),
        auto_continue=True
    )
    instructions.append(control_mode)
    instruction_id += 1
    
    # Step 2: Set pipeline inspection depth (0.5 meters)
    pipeline_depth_set_point = bp.DepthSetPoint(
        depth=0.1,  # Shallow depth for pipeline inspection
        speed_to_depth=0.2,  # 0.1 m/s
        depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE
    )
    
    goto_pipeline_depth = bp.Instruction(
        id=instruction_id,
        depth_set_point_command=bp.DepthSetPointCommand(
            depth_set_point=pipeline_depth_set_point
        ),
        auto_continue=True
    )
    instructions.append(goto_pipeline_depth)
    instruction_id += 1
    
    # Step 3-6: Navigate to each pipeline waypoint
    for point in pipeline_waypoints:
        waypoint = bp.Waypoint(
            id=instruction_id,
            name=point["name"],
            global_position=LatLongPosition(
                latitude=point["lat"],
                longitude=point["lon"]
            ),
            circle_of_acceptance=1.0,  # 1 meter acceptance radius
            speed_to_target=0.5,  # 0.3 m/s
            depth_set_point=pipeline_depth_set_point
        )
        
        goto_waypoint = bp.Instruction(
            id=instruction_id,
            waypoint_command=bp.WaypointCommand(
                waypoint=waypoint
            ),
            auto_continue=True
        )
        instructions.append(goto_waypoint)
        instruction_id += 1
        logger.info(f"Added waypoint: {point['name']} ({point['lat']}, {point['lon']}) at 0.5m depth")
        
        # Add a wait instruction to stabilize at each waypoint (optional)
        wait_instruction = bp.Instruction(
            id=instruction_id,
            wait_for_command=bp.WaitForCommand(
                wait_for_seconds=5.0  # Wait 5 seconds at each waypoint
            ),
            auto_continue=True
        )
        instructions.append(wait_instruction)
        instruction_id += 1
    
    # Step 7: Set docking depth
    docking_depth_set_point = bp.DepthSetPoint(
        depth=docking_station["depth"],
        speed_to_depth=0.5,  # 0.2 m/s
        depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE
    )
    
    # Step 8: Navigate to docking station
    docking_waypoint = bp.Waypoint(
        id=instruction_id,
        name=docking_station["name"],
        global_position=LatLongPosition(
            latitude=docking_station["lat"],
            longitude=docking_station["lon"]
        ),
        circle_of_acceptance=0.5,  # 0.5 meter acceptance radius (more precise for docking)
        speed_to_target=0.3,  # Slower approach speed for docking
        depth_set_point=docking_depth_set_point
    )
    
    goto_docking = bp.Instruction(
        id=instruction_id,
        waypoint_command=bp.WaypointCommand(
            waypoint=docking_waypoint
        ),
        auto_continue=True
    )
    instructions.append(goto_docking)
    instruction_id += 1
    logger.info(f"Added docking waypoint: ({docking_station['lat']}, {docking_station['lon']}) at {docking_station['depth']}m depth")
    
    # Create the mission
    mission = bp.Mission(
        id=1,
        name="Pipeline Survey",
        instructions=instructions,
        default_surge_speed=0.3,
        default_heave_speed=0.2,
        default_circle_of_acceptance=1.0
    )
    
    return mission


def run_mission(drone, mission, max_duration, logger):
    """Run the mission and monitor its progress."""
    if not drone or not drone.connected:
        logger.error("Drone not connected. Cannot run mission.")
        return False
    
    start_time = time.time()
    
    try:
        # Clear any existing missions
        logger.info("Clearing any previous missions")
        drone.mission.clear()
        time.sleep(1)
        
        # Send the new mission
        logger.info(f"Sending new mission: {mission.name}")
        drone.mission.send_new(mission)
        time.sleep(1)
        
        # Check if the mission was loaded successfully
        status = drone.mission.get_status()
        if status.state != bp.MissionState.MISSION_STATE_READY:
            logger.error(f"Failed to load mission. State: {status.state.name}")
            return False
        
        # Start the mission
        logger.info("Starting mission execution")
        drone.mission.run()
        
        # Monitor mission progress
        while True:
            # Check if maximum duration exceeded
            elapsed = time.time() - start_time
            if elapsed > max_duration:
                logger.warning(f"Mission timeout after {elapsed:.1f} seconds")
                drone.mission.stop()
                return False
            
            # Get current mission status
            status = drone.mission.get_status()
            
            # Log current status
            state_msg = f"Mission: {status.state.name}, "
            state_msg += f"Progress: {len(status.completed_instruction_ids)}/{status.total_number_of_instructions} instructions, "
            state_msg += f"Time: {status.time_elapsed}s/{status.time_elapsed + status.estimated_time_to_complete}s"
            try:
                depth = drone.depth
                state_msg += f", Depth: {depth:.1f}m"
            except:
                pass
            logger.info(state_msg)
            
            # Check mission state
            if status.state == bp.MissionState.MISSION_STATE_COMPLETED:
                logger.info("Mission completed successfully")
                return True
            elif status.state == bp.MissionState.MISSION_STATE_ABORTED:
                logger.warning("Mission was aborted")
                return False
            elif status.state in [
                bp.MissionState.MISSION_STATE_FAILED_TO_LOAD_MISSION,
                bp.MissionState.MISSION_STATE_FAILED_TO_START_MISSION
            ]:
                logger.error(f"Mission failed with state: {status.state.name}")
                return False
            
            time.sleep(2)  # Poll every 2 seconds
            
    except Exception as e:
        logger.error(f"Error during mission execution: {str(e)}")
        # Try to stop the mission if there's an error
        try:
            drone.mission.stop()
        except:
            pass
        return False


def disconnect_drone(drone, logger):
    """Disconnect from the drone safely."""
    if not drone or not drone.connected:
        return True
    
    logger.info("Disconnecting from drone")
    try:
        # Stop mission if still running
        try:
            status = drone.mission.get_status()
            if status and status.state and status.state.name == "MISSION_STATE_RUNNING":
                logger.info("Stopping active mission")
                drone.mission.stop()
                time.sleep(1)
        except Exception as e:
            logger.warning(f"Error checking mission status: {str(e)}")
        
        # Disconnect
        drone.disconnect()
        logger.info("Disconnected from drone")
        return True
        
    except Exception as e:
        logger.error(f"Error during disconnect: {str(e)}")
        return False


def main():
    """Main function."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Set up logging
    logger = setup_logging()
    logger.info("Starting Pipeline Survey and Docking Mission with Position Reset")
    
    drone = None
    
    try:
        # Connect to the drone
        drone = connect_to_drone(args.drone_ip, args.timeout, logger)
        if not drone:
            logger.error("Failed to connect to drone. Exiting.")
            return 1
        
        # Reset the drone's position to the provided starting coordinates
        if not reset_drone_position(drone, args.start_lat, args.start_lon, args.start_heading, logger):
            logger.error("Failed to reset drone position. Exiting.")
            return 1
        
        # Create the mission
        mission = create_mission(logger)
        
        # Run the mission
        success = run_mission(drone, mission, 18000, logger)  # 30 minutes max
        
        logger.info(f"Mission {'completed successfully' if success else 'failed'}")
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("Mission aborted by user")
        return 130  # Standard exit code for SIGINT
        
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        return 1
        
    finally:
        # Always try to disconnect from the drone
        if drone:
            disconnect_drone(drone, logger)
            
        if success:
            logger.info(f"Mission completed successfully")
            return 0  # Success
        else:
            logger.info(f"Mission failed")
            return 1  # Failure


if __name__ == "__main__":
    sys.exit(main())
    
    
 # pipeline_waypoints = [
    # {"lat": 63.4406850, "lon": 10.3489600, "name": "Pipeline Point 1"},
    # {"lat": 63.4406650, "lon": 10.3489300, "name": "Pipeline Point 2"},
    # {"lat": 63.4406700, "lon": 10.3489800, "name": "Pipeline Point 3"},
    # {"lat": 63.4406850, "lon": 10.3489850, "name": "Pipeline Point 4"},
    # ]
    # docking_station = {
    # "lat": 63.4406991,
    # "lon": 10.3489964,
    # "name": "Docking Station",
    # "depth": 2.3
    # }
    
    
# pipeline_inspection.py  --drone-ip 192.168.1.101 --start-lat 63.4406996 --start-lon 10.3489307 --start-heading 0.0