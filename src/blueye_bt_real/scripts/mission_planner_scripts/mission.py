#!/usr/bin/env python3
"""
Pool Test Mission Script for Blueye Drone with Position Reset and Auto Resume

This script executes an autonomous pipeline inspection and docking mission with 3 waypoints:
1-3. Navigate along pipeline waypoints at surface depth (0.0m)
4. Navigate to the docking station at surface depth (0.0m)

Usage:
  python mission_with_auto_resume.py [--drone-ip IP_ADDRESS] [--timeout SECONDS]
"""
import json
import argparse
import sys
import time
import logging
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition


def extend_ctrl_client(drone, logger=None, use_hardcoded_gps=True):
    """
    Extend the CtrlClient class of a connected drone with a reset_position method.
    Args:
        drone (Drone): The connected Blueye drone.
        logger: Logger instance (optional).
        use_hardcoded_gps (bool): If True, use hardcoded GPS. If False, use GPS topic.
    Returns:
        The drone with extended functionality.
    """
    # Hardcoded GPS coordinates
    HARDCODED_LAT = 63.4414548287786
    HARDCODED_LON = 10.3482882678509
    
    def reset_position(lat=None, lon=None, heading=None):
        """
        Reset the drone's position to a specified GPS coordinate.
        Args:
            lat (float, optional): Latitude in decimal degrees.
            lon (float, optional): Longitude in decimal degrees.
            heading (float, optional): Heading in degrees (0-359). If None, uses drone compass.
        """
        # If lat/lon not provided, use hardcoded or get from topic based on flag
        if lat is None or lon is None:
            if use_hardcoded_gps:
                # Use hardcoded coordinates
                lat, lon = HARDCODED_LAT, HARDCODED_LON
                if logger:
                    logger.info(f"Using hardcoded GPS position: {lat}, {lon}")
            else:
                # Use GPS topic
                if logger:
                    logger.info("Getting current GPS position from /blueye/gps topic")
                
                # Initialize ROS if not already done
                if not rclpy.ok():
                    rclpy.init()
                
                # Create temporary node and get one GPS message
                temp_node = Node('temp_gps_reader')
                
                try:
                    msg = None
                    def gps_callback(data):
                        nonlocal msg
                        msg = data
                    
                    subscription = temp_node.create_subscription(Point, '/blueye/gps', gps_callback, 10)
                    
                    # Spin once to get the message
                    while msg is None:
                        rclpy.spin_once(temp_node, timeout_sec=0.1)
                    
                    lat, lon = msg.x, msg.y
                    if logger:
                        logger.info(f"Using current GPS position: {lat}, {lon}")
                    
                    temp_node.destroy_node()
                    
                except Exception as e:
                    if logger:
                        logger.error(f"Failed to get GPS position: {str(e)}")
                    temp_node.destroy_node()
                    raise e
        
        # Create reset position settings
        reset_settings = {
            "heading_source_during_reset": bp.HeadingSource.HEADING_SOURCE_MANUAL_INPUT if heading is not None else bp.HeadingSource.HEADING_SOURCE_DRONE_COMPASS,
            "manual_heading": heading if heading is not None else 0.0,
            "reset_coordinate_source": bp.ResetCoordinateSource.RESET_COORDINATE_SOURCE_MANUAL,
            "reset_coordinate": {
                "latitude": lat,
                "longitude": lon
            },
        }
        
        # Create and send the reset position message
        reset_msg = bp.ResetPositionCtrl(settings=reset_settings)
        drone._ctrl_client._messages_to_send.put(reset_msg)
        
        if logger:
            heading_source = "drone compass" if heading is None else f"manual ({heading}°)"
            logger.info(f"Position reset sent: lat={lat}, lon={lon}, heading={heading_source}")
    
    # Attach the method to the CtrlClient
    drone._ctrl_client.reset_position = reset_position
    return drone

def setup_logging(log_file="log/mission.log", log_level="INFO"):
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
    
    # Mission retry settings
    parser.add_argument('--max-retries', type=int, default=5,
                        help='Maximum number of retry attempts for aborted missions')
    
    # Parse arguments
    return parser.parse_args()


def connect_to_drone(ip, timeout, logger):
    """Connect to the drone."""
    logger.info(f"Connecting to drone at {ip}")
    try:
        drone = Drone(ip=ip, auto_connect=True, timeout=timeout)
        
        logger.info(f"Connected to drone: {drone.serial_number}")
        logger.info(f"Drone software version: {drone.software_version}")
        
        if not drone.in_control:
            logger.info("Taking control of drone...")
            drone.take_control()
            logger.info("Control of drone acquired")
        
        # Extend the drone with the reset_position functionality
        # Set use_hardcoded_gps=True to use hardcoded coordinates
        # Set use_hardcoded_gps=False to use GPS topic
        drone = extend_ctrl_client(drone, logger, use_hardcoded_gps=True)
        logger.info("Drone control client extended with reset_position functionality")
        
        return drone
        
    except Exception as e:
        logger.error(f"Failed to connect to drone: {str(e)}")
        return None


def reset_drone_position(drone, logger):
    """Reset the drone's position using current GPS coordinates."""
    try:
        drone._ctrl_client.reset_position()
        logger.info("Position reset successful using current GPS and drone compass")
        time.sleep(2)
        return True
    except Exception as e:
        logger.error(f"Failed to reset drone position: {str(e)}")
        return False


def create_mission(logger):
    """Create a mission with pipeline survey waypoints and docking station."""
    logger.info("Creating pipeline survey")
    
    pipeline_waypoints = [
        {"lat": 63.4414001130402, "lon": 10.348227918148, "name": "Pipeline Point 1"},
        {"lat": 63.4413996633213, "lon": 10.3483355417848, "name": "Pipeline Point 2"},
        {"lat": 63.4414320430591, "lon": 10.3482845798135, "name": "Pipeline Point 3"},
    ]
    docking_station = {
        "lat": 63.4414548287786,
        "lon": 10.3482882678509,
        "name": "Docking Station",
        "depth": 1.0
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
    
    # Step 2: Set pipeline inspection depth (0.0 meters - surface)
    pipeline_depth_set_point = bp.DepthSetPoint(
        # ADD DEPTH TO PIPELINE WAYPOINTS
        depth=1.0, 
        speed_to_depth=0.5,
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
            circle_of_acceptance=0.5,
            speed_to_target=0.2,
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
        logger.info(f"Added waypoint: {point['name']} ({point['lat']}, {point['lon']}) at 0.0m depth")
        
        # Add a wait instruction to stabilize at each waypoint (optional)
        wait_instruction = bp.Instruction(
            id=instruction_id,
            wait_for_command=bp.WaitForCommand(
                wait_for_seconds=10.0
            ),
            auto_continue=True
        )
        instructions.append(wait_instruction)
        instruction_id += 1
    
    # Step 7: Set docking depth
    docking_depth_set_point = bp.DepthSetPoint(
        depth=docking_station["depth"],
        speed_to_depth=0.5, 
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
        circle_of_acceptance=0.5,  
        speed_to_target=0.5, 
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
        default_surge_speed=0.2,
        default_heave_speed=0.5,
        default_circle_of_acceptance=0.5
    )
    
    return mission


def save_mission_to_json(mission, filename="mission.json", logger=None):
    """Save the mission to a JSON file for cross-checking."""
    try:
        # Convert mission to dictionary format
        mission_dict = {
            "id": mission.id,
            "name": mission.name,
            "default_surge_speed": mission.default_surge_speed,
            "default_heave_speed": mission.default_heave_speed,
            "default_circle_of_acceptance": mission.default_circle_of_acceptance,
            "instructions": []
        }
        
        # Convert each instruction to dictionary
        for instruction in mission.instructions:
            instr_dict = {
                "id": instruction.id,
                "auto_continue": instruction.auto_continue
            }
            
            # Handle different instruction types
            if instruction.control_mode_command:
                instr_dict["type"] = "control_mode"
                instr_dict["control_mode_vertical"] = instruction.control_mode_command.control_mode_vertical.name
                instr_dict["control_mode_horizontal"] = instruction.control_mode_command.control_mode_horizontal.name
                
            elif instruction.depth_set_point_command:
                instr_dict["type"] = "depth_set_point"
                instr_dict["depth"] = instruction.depth_set_point_command.depth_set_point.depth
                instr_dict["speed_to_depth"] = instruction.depth_set_point_command.depth_set_point.speed_to_depth
                instr_dict["depth_zero_reference"] = instruction.depth_set_point_command.depth_set_point.depth_zero_reference.name
                
            elif instruction.waypoint_command:
                instr_dict["type"] = "waypoint"
                waypoint = instruction.waypoint_command.waypoint
                instr_dict["name"] = waypoint.name
                instr_dict["latitude"] = waypoint.global_position.latitude
                instr_dict["longitude"] = waypoint.global_position.longitude
                instr_dict["circle_of_acceptance"] = waypoint.circle_of_acceptance
                instr_dict["speed_to_target"] = waypoint.speed_to_target
                instr_dict["depth"] = waypoint.depth_set_point.depth
                
            elif instruction.wait_for_command:
                instr_dict["type"] = "wait"
                instr_dict["wait_for_seconds"] = instruction.wait_for_command.wait_for_seconds
            
            mission_dict["instructions"].append(instr_dict)
        
        # Save to JSON file
        with open(filename, 'w') as f:
            json.dump(mission_dict, f, indent=2)
        
        if logger:
            logger.info(f"Mission saved to {filename}")
        else:
            print(f"Mission saved to {filename}")
            
        return True
        
    except Exception as e:
        error_msg = f"Failed to save mission to JSON: {str(e)}"
        if logger:
            logger.error(error_msg)
        else:
            print(error_msg)
        return False


def run_mission_with_instant_resume(drone, mission, max_duration, logger, max_retries=10):
    """Run the mission and instantly retry if aborted."""
    if not drone or not drone.connected:
        logger.error("Drone not connected. Cannot run mission.")
        return False
    
    start_time = time.time()
    retry_count = 0
    
    try:
        # Clear any existing missions
        logger.info("Clearing any previous missions")
        drone.mission.clear()
        time.sleep(1)
        
        # Send the new mission
        logger.info(f"Sending new mission: {mission.name}")
        drone.mission.send_new(mission)
        time.sleep(1)
        
        # Main mission execution loop with retries
        while retry_count <= max_retries:
            # Start the mission
            if retry_count == 0:
                logger.info("Starting mission execution")
            else:
                logger.info(f"Instantly resuming mission execution (attempt {retry_count + 1}/{max_retries + 1})")
            
            drone.mission.run()  # This will automatically continue from where it stopped
            
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
                    logger.warning(f"Mission was aborted (attempt {retry_count + 1}/{max_retries + 1})")
                    
                    if retry_count < max_retries:
                        # Instantly retry
                        retry_count += 1
                        break  # Break out of the inner loop to retry mission.run()
                    else:
                        logger.warning(f"Maximum retry attempts ({max_retries}) reached. Mission failed.")
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
    logger.info("Starting Pipeline Survey and Docking Mission with Position Reset and Auto Resume")
    
    drone = None
    success = False
    
    try:
        # Connect to the drone
        drone = connect_to_drone(args.drone_ip, args.timeout, logger)
        if not drone:
            logger.error("Failed to connect to drone. Exiting.")
            return 1
        
        if not reset_drone_position(drone, logger):
            logger.error("Failed to reset drone position. Exiting.")
            return 1
        
        # Create the mission
        mission = create_mission(logger)
        
        save_mission_to_json(mission, "mission.json", logger)
        
        # Run the mission with instant auto-resume
        success = run_mission_with_instant_resume(drone, mission, 1800, logger, args.max_retries)
        
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
        
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())