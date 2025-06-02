#!/usr/bin/env python3
"""
Single Waypoint Mission Script for Blueye Drone with connection reuse and integrated position reset.

This script checks for an existing connection, maintains it across multiple waypoints,
and includes position reset functionality without external dependencies.
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

# ROS2 imports for GPS functionality
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

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

def extend_ctrl_client_with_reset(drone, logger=None):
    """
    Extend the CtrlClient class of a connected drone with a reset_position method.
    Args:
        drone (Drone): The connected Blueye drone.
        logger: Logger instance (optional).
    Returns:
        The drone with extended functionality.
    """
    def reset_position(lat=None, lon=None, heading=None):
        """
        Reset the drone's position to a specified GPS coordinate.
        Args:
            lat (float, optional): Latitude in decimal degrees. If None, gets from /blueye/gps topic.
            lon (float, optional): Longitude in decimal degrees. If None, gets from /blueye/gps topic.
            heading (float, optional): Heading in degrees (0-359). If None, uses drone compass.
        """
        # If lat/lon not provided, get current position from ROS topic
        if lat is None or lon is None:
            if logger:
                logger.info("Getting current GPS position from /blueye/gps topic")
            
            # Initialize ROS if not already done
            if not rclpy.ok():
                rclpy.init()
            
            # Create temporary node and get one GPS message
            temp_node = Node('temp_gps_reader')
            
            try:
                # Get one GPS message (since it's publishing at 10Hz, this will be quick)
                msg = None
                def gps_callback(data):
                    nonlocal msg
                    msg = data
                
                subscription = temp_node.create_subscription(Point, '/blueye/gps', gps_callback, 10)
                
                # Spin once to get the message with timeout
                start_time = time.time()
                timeout = 5.0  # 5 second timeout
                while msg is None and (time.time() - start_time) < timeout:
                    rclpy.spin_once(temp_node, timeout_sec=0.1)
                
                if msg is None:
                    raise Exception("Timeout waiting for GPS message from /blueye/gps topic")
                
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

def get_or_create_connection(drone_ip, initialize=False, logger=None):
    """Create a new drone connection."""
    if logger is None:
        logger = setup_logging()
    
    logger.info(f"Creating connection to drone at {drone_ip}")
    try:
        # Connect to the drone
        drone = Drone(ip=drone_ip, auto_connect=True)
        logger.info(f"Connected to drone: {drone.serial_number}")
        
        # Take control if needed
        if not drone.in_control:
            drone.take_control()
            logger.info("Control of drone acquired")
        
        # Extend the drone with the reset_position functionality
        drone = extend_ctrl_client_with_reset(drone, logger)
        
        # Reset position when requested
        if initialize:
            logger.info("Resetting drone position to current GPS coordinates")
            try:
                drone._ctrl_client.reset_position()
                time.sleep(2)  # Brief pause after reset
                logger.info("Position reset completed successfully")
            except Exception as e:
                logger.error(f"Position reset failed: {str(e)}")
                return None
        
        return drone
    
    except Exception as e:
        logger.error(f"Failed to connect to drone: {str(e)}")
        return None

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
    
    goto_waypoint_instr = bp.Instruction(
        id=3,
        waypoint_command=bp.WaypointCommand(
            waypoint=waypoint
        ),
        auto_continue=True
    )
    instructions.append(goto_waypoint_instr)
    
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
            depth_current = drone.depth
            state_msg += f", Depth: {depth_current:.1f}m"
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
    drone = None
    success = False
    
    try:
        # Command line arguments
        if len(sys.argv) < 6:
            print("Usage: python goto_waypoint_cc.py <waypoint_lat> <waypoint_lon> <depth> <drone_ip> <initialize_connection>")
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
            logger.error("Failed to get drone connection or position reset failed")
            return 1
        
        # Navigate to waypoint
        success = goto_waypoint(drone, waypoint_lat, waypoint_lon, depth, logger)
        
        logger.info(f"Mission {'completed successfully' if success else 'failed'}")
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("Mission aborted by user")
        return 130  # Standard exit code for SIGINT
        
    except Exception as e:
        if 'logger' in locals():
            logger.error(f"Unexpected error: {str(e)}", exc_info=True)
        else:
            print(f"Unexpected error: {str(e)}")
        return 1
        
    finally:
        # Note: We don't disconnect - the connection will be reused for performance
        # Only log the final status
        if 'logger' in locals():
            logger.info("Script execution completed")

if __name__ == "__main__":
    sys.exit(main())