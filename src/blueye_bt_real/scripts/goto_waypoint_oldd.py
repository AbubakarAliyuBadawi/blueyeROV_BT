#!/usr/bin/env python3
"""
Reset Position and Navigate to Waypoint Script for Blueye Drone

This script:
1. Resets the drone's position to specific coordinates and heading
2. Navigates to the specified waypoint at a designated depth

Usage:
  python reset_and_goto_waypoint.py [waypoint_lat] [waypoint_lon] [depth] [drone_ip]
"""

import sys
import time
import logging
import argparse

# Import required Blueye SDK components
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition

# Position reset functionality
def extend_ctrl_client(drone):
    """
    Extend the CtrlClient class of a connected drone with a reset_position method.
    Args:
    drone (Drone): The connected Blueye drone.
    Returns:
    The drone with extended functionality.
    """
    # Add the reset_position method to the CtrlClient instance
    def reset_position(lat, lon, heading=0.0):
        """
        Reset the drone's position to a specified GPS coordinate.
        Args:
        lat (float): Latitude in decimal degrees.
        lon (float): Longitude in decimal degrees.
        heading (float, optional): Heading in degrees (0-359). Defaults to 0.0.
        """
        # Create reset position settings
        reset_settings = {
            "heading_source_during_reset": bp.HeadingSource.HEADING_SOURCE_MANUAL_INPUT if heading != 0.0 else bp.HeadingSource.HEADING_SOURCE_DRONE_COMPASS,
            "manual_heading": heading,
            "reset_coordinate_source": bp.ResetCoordinateSource.RESET_COORDINATE_SOURCE_MANUAL,
            "reset_coordinate": {
                "latitude": lat,
                "longitude": lon
            },
        }
        # Create and send the reset position message
        msg = bp.ResetPositionCtrl(settings=reset_settings)
        drone._ctrl_client._messages_to_send.put(msg)
    
    # Attach the method to the CtrlClient
    drone._ctrl_client.reset_position = reset_position
    return drone

def setup_logging():
    """Set up basic logging to console."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("reset_and_waypoint")

def parse_arguments():
    """Parse command line arguments with defaults."""
    parser = argparse.ArgumentParser(description='Reset position and go to waypoint')
    
    # Waypoint arguments
    parser.add_argument('waypoint_lat', type=float, nargs='?', default=63.4406620,
                        help='Target waypoint latitude')
    parser.add_argument('waypoint_lon', type=float, nargs='?', default=10.3488502,
                        help='Target waypoint longitude')
    parser.add_argument('depth', type=float, nargs='?', default=1.0,
                        help='Target depth in meters')
    parser.add_argument('drone_ip', type=str, nargs='?', default="192.168.1.101",
                        help='IP address of the drone')
    
    # Reset position arguments (hardcoded to the requested values)
    parser.add_argument('--reset-lat', type=float, default=63.4406620,
                        help='Latitude for position reset')
    parser.add_argument('--reset-lon', type=float, default=10.3488502,
                        help='Longitude for position reset')
    parser.add_argument('--reset-heading', type=float, default=163,
                        help='Heading in degrees for position reset')
    
    # Mission settings
    parser.add_argument('--max-retries', type=int, default=3,
                        help='Maximum number of retry attempts')
    
    return parser.parse_args()

def main():
    # Parse arguments
    args = parse_arguments()
    
    # Set up logging
    logger = setup_logging()
    
    # Extract values
    waypoint_lat = args.waypoint_lat
    waypoint_lon = args.waypoint_lon
    depth = args.depth
    drone_ip = args.drone_ip
    reset_lat = args.reset_lat
    reset_lon = args.reset_lon
    reset_heading = args.reset_heading
    max_retries = args.max_retries
    
    logger.info(f"Starting mission to waypoint ({waypoint_lat}, {waypoint_lon}) at {depth}m depth")
    logger.info(f"Will first reset position to ({reset_lat}, {reset_lon}) with heading {reset_heading}°")
    
    try:
        # Connect to the drone
        logger.info(f"Connecting to drone at {drone_ip}")
        drone = Drone(ip=drone_ip, auto_connect=True)
        logger.info(f"Connected to drone: {drone.serial_number}")
        
        # Take control if needed
        if not drone.in_control:
            drone.take_control()
            logger.info("Control of drone acquired")
        
        # Reset position
        logger.info(f"Resetting drone position to: {reset_lat}, {reset_lon}, heading: {reset_heading}°")
        drone = extend_ctrl_client(drone)
        drone._ctrl_client.reset_position(reset_lat, reset_lon, reset_heading)
        logger.info("Position reset command sent")
        time.sleep(5)  # Wait for position reset to take effect
        
        # Create single waypoint mission
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
                return 0
            
            # Auto-resume if aborted
            elif status.state == bp.MissionState.MISSION_STATE_ABORTED:
                if retry_count < max_retries:
                    retry_count += 1
                    logger.info(f"Mission aborted, auto-resuming (attempt {retry_count}/{max_retries})")
                    drone.mission.run()  # Resume mission
                else:
                    logger.warning("Maximum retry attempts reached")
                    return 1
            
            # Handle other failure cases
            elif status.state in [
                bp.MissionState.MISSION_STATE_FAILED_TO_LOAD_MISSION,
                bp.MissionState.MISSION_STATE_FAILED_TO_START_MISSION
            ]:
                logger.error(f"Mission failed with state: {status.state.name}")
                return 1
                
            time.sleep(2)  # Check status every 2 seconds
        
    except Exception as e:
        logger.error(f"Error: {str(e)}")
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)