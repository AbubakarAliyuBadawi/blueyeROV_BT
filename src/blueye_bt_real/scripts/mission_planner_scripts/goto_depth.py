#!/usr/bin/env python3
"""
Simple Depth Mission Script for Blueye Drone with Position Reset

This script executes a simple mission where the drone:
1. Sets its initial position using the reset_position functionality
2. Descends to a target depth of 1.3 meters (referenced from the surface)
3. Maintains that depth for a specified duration

Usage:
  python goto_depth.py [--drone-ip IP_ADDRESS] [--timeout SECONDS] [--duration SECONDS]
"""

import argparse
import sys
import time
import logging
from pathlib import Path

# Import required Blueye SDK components
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference

# Import our position reset extension
from reset_drone_position import extend_ctrl_client


def setup_logging(log_file="log/goto_dept.log", log_level="INFO"):
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
    
    return logging.getLogger("depth_mission")


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Run a simple depth mission with position reset')
    
    # Drone settings
    parser.add_argument('--drone-ip', type=str, default="192.168.1.101",
                        help='IP address of the Blueye drone')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Connection timeout for the drone in seconds')
    
    # Mission settings
    parser.add_argument('--depth', type=float, default=1.3,
                        help='Target depth in meters (from surface)')
    parser.add_argument('--duration', type=int, default=120,
                        help='Duration to maintain depth in seconds')
    
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
        time.sleep(5)
        return True
    except Exception as e:
        logger.error(f"Failed to reset drone position: {str(e)}")
        return False


def create_simple_depth_mission(depth, duration, logger):
    """Create a simple mission to descend to a specific depth and maintain it."""
    logger.info(f"Creating simple depth mission to {depth}m for {duration} seconds")
    
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
    
    # Step 2: Set target depth
    depth_set_point = bp.DepthSetPoint(
        depth=depth,  # Our target depth
        speed_to_depth=0.2,  # 0.2 m/s descent rate
        depth_zero_reference=DepthZeroReference.DEPTH_ZERO_REFERENCE_SURFACE
    )
    
    goto_depth = bp.Instruction(
        id=instruction_id,
        depth_set_point_command=bp.DepthSetPointCommand(
            depth_set_point=depth_set_point
        ),
        auto_continue=True
    )
    instructions.append(goto_depth)
    instruction_id += 1
    
    # Step 3: Wait at depth for specified duration
    wait_instruction = bp.Instruction(
        id=instruction_id,
        wait_for_command=bp.WaitForCommand(
            wait_for_seconds=float(duration)  # Wait for the specified duration
        ),
        auto_continue=True
    )
    instructions.append(wait_instruction)
    instruction_id += 1
    
    # Create the mission
    mission = bp.Mission(
        id=1,
        name="Simple Depth Mission",
        instructions=instructions,
        default_surge_speed=0.2,
        default_heave_speed=0.3,
        default_circle_of_acceptance=0.5
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
    logger.info(f"Starting Simple Depth Mission to {args.depth}m for {args.duration}s")
    
    drone = None
    success = False
    
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
        mission = create_simple_depth_mission(args.depth, args.duration, logger)
        
        # Run the mission
        success = run_mission(drone, mission, args.duration + 120, logger)  # Add extra time for setup
        
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


if __name__ == "__main__":
    sys.exit(main())
    
    
# python goto_depth.py --depth 1.5 --duration 180 --start-lat 63.4406991 --start-lon 10.3489964
