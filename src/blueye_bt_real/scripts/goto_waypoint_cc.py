#!/usr/bin/env python3
"""
Single Waypoint Mission Script for Blueye Drone with connection reuse.

This script checks for an existing connection and maintains it across multiple waypoints.
"""

import sys
import time
import os
import pickle
import logging

# Import required Blueye SDK components
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition

# Import position reset extension
from mission_planner_scripts.reset_drone_position import extend_ctrl_client

# File path for storing the connection object
CONNECTION_CACHE_FILE = "/tmp/blueye_drone_connection.pkl"

def setup_logging():
    """Set up basic logging to console."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("waypoint_mission")

def get_or_create_connection(drone_ip, initialize=False, logger=None):
    """Get an existing drone connection or create a new one."""
    if logger is None:
        logger = setup_logging()
    
    # If initialize is True or if no connection exists, create a new one
    if initialize or not os.path.exists(CONNECTION_CACHE_FILE):
        logger.info(f"Creating new connection to drone at {drone_ip}")
        try:
            # Connect to the drone
            drone = Drone(ip=drone_ip, auto_connect=True)
            logger.info(f"Connected to drone: {drone.serial_number}")
            
            # Take control if needed
            if not drone.in_control:
                drone.take_control()
                logger.info("Control of drone acquired")
            
            # Extend the drone with the reset_position functionality
            drone = extend_ctrl_client(drone)
            
            # Save the connection to file
            with open(CONNECTION_CACHE_FILE, 'wb') as f:
                pickle.dump(drone, f)
            
            return drone
        
        except Exception as e:
            logger.error(f"Failed to connect to drone: {str(e)}")
            return None
    
    else:
        # Try to load existing connection
        logger.info("Attempting to use existing drone connection")
        try:
            with open(CONNECTION_CACHE_FILE, 'rb') as f:
                drone = pickle.load(f)
            
            # Verify connection is still active
            if drone.connected:
                logger.info("Using existing drone connection")
                return drone
            else:
                logger.warning("Existing connection is no longer active, creating new one")
                os.remove(CONNECTION_CACHE_FILE)
                return get_or_create_connection(drone_ip, True, logger)
        
        except Exception as e:
            logger.warning(f"Failed to load existing connection: {str(e)}")
            os.remove(CONNECTION_CACHE_FILE)
            return get_or_create_connection(drone_ip, True, logger)

def goto_waypoint(drone, waypoint_lat, waypoint_lon, depth, logger):
    """Navigate to the specified waypoint."""
    logger.info(f"Creating waypoint mission to ({waypoint_lat}, {waypoint_lon}) at {depth}m depth")
    
    # Create the instructions for the mission
    instructions = []
    
    # Step 1: Configure auto-depth mode
    control_mode = bp.Instruction(
        id=1,
        control_mode_command=bp.ControlModeCommand(
            control_mode_vertical=bp.ControlModeVertical.CONTROL_MODE_VERTICAL_AUTO_DEPTH,
            control_mode_horizontal=bp.ControlModeHorizontal.CONTROL_MODE_HORIZONTAL_AUTO_HEADING
        ),
        auto_continue=True
    )
    instructions.append(control_mode)
    
    # Step 2: Set depth
    depth_set_point = bp.DepthSetPoint(
        depth=depth,
        speed_to_depth=0.2,
        depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE
    )
    
    goto_depth = bp.Instruction(
        id=2,
        depth_set_point_command=bp.DepthSetPointCommand(
            depth_set_point=depth_set_point
        ),
        auto_continue=True
    )
    instructions.append(goto_depth)
    
    # Step 3: Navigate to waypoint
    waypoint = bp.Waypoint(
        id=3,
        name="Target Waypoint",
        global_position=LatLongPosition(
            latitude=waypoint_lat,
            longitude=waypoint_lon
        ),
        circle_of_acceptance=1.0,
        speed_to_target=0.3,
        depth_set_point=depth_set_point
    )
    
    goto_waypoint = bp.Instruction(
        id=3,
        waypoint_command=bp.WaypointCommand(
            waypoint=waypoint
        ),
        auto_continue=True
    )
    instructions.append(goto_waypoint)
    
    # Create the mission
    mission = bp.Mission(
        id=1,
        name="Waypoint Mission",
        instructions=instructions,
        default_surge_speed=0.3,
        default_heave_speed=0.2,
        default_circle_of_acceptance=1.0
    )
    
    # Clear existing missions
    logger.info("Clearing any previous missions")
    drone.mission.clear()
    time.sleep(1)
    
    # Send and run the mission
    logger.info("Sending and running waypoint mission")
    drone.mission.send_new(mission)
    time.sleep(1)
    drone.mission.run()
    
    # Monitor mission progress with auto-resume
    retry_count = 0
    max_retries = 3
    
    while retry_count <= max_retries:
        # Get mission status
        status = drone.mission.get_status()
        
        # Log status
        state_msg = f"Mission: {status.state.name}, "
        state_msg += f"Progress: {len(status.completed_instruction_ids)}/{status.total_number_of_instructions} instructions"
        try:
            depth = drone.depth
            state_msg += f", Depth: {depth:.1f}m"
        except:
            pass
        logger.info(state_msg)
        
        # Check if mission completed
        if status.state == bp.MissionState.MISSION_STATE_COMPLETED:
            logger.info("Mission completed successfully")
            return True
        
        # Auto-resume if aborted
        elif status.state == bp.MissionState.MISSION_STATE_ABORTED:
            if retry_count < max_retries:
                retry_count += 1
                logger.info(f"Mission aborted, auto-resuming (attempt {retry_count}/{max_retries})")
                drone.mission.run()  # Resume mission
            else:
                logger.warning("Maximum retry attempts reached")
                return False
        
        # Handle other failure cases
        elif status.state in [
            bp.MissionState.MISSION_STATE_FAILED_TO_LOAD_MISSION,
            bp.MissionState.MISSION_STATE_FAILED_TO_START_MISSION
        ]:
            logger.error(f"Mission failed with state: {status.state.name}")
            return False
            
        time.sleep(2)  # Check status every 2 seconds
    
    # We should never reach here, but just in case
    return False

def main():
    """Main function."""
    # Command line arguments
    if len(sys.argv) < 6:
        print("Usage: python goto_waypoint.py <waypoint_lat> <waypoint_lon> <depth> <drone_ip> <initialize_connection>")
        return 1
    
    waypoint_lat = float(sys.argv[1])
    waypoint_lon = float(sys.argv[2])
    depth = float(sys.argv[3])
    drone_ip = sys.argv[4]
    initialize_connection = sys.argv[5].lower() in ['true', '1', 't', 'yes']
    
    # Set up logging
    logger = setup_logging()
    
    # Get or create drone connection
    drone = get_or_create_connection(drone_ip, initialize_connection, logger)
    if not drone:
        logger.error("Failed to get drone connection")
        return 1
    
    # Navigate to waypoint
    success = goto_waypoint(drone, waypoint_lat, waypoint_lon, depth, logger)
    
    # Note: We don't disconnect - the connection will be reused
    
    # Return success or failure
    return 0 if success else 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)