#!/usr/bin/env python3
"""
Single Waypoint Mission Script for Blueye Drone

This script executes a simple mission to navigate to a single waypoint.
It doesn't disconnect from the drone when finished.
"""

import sys
import time
import logging

# Import required Blueye SDK components
import blueye.protocol as bp
from blueye.sdk import Drone
from blueye.protocol.types.mission_planning import DepthZeroReference
from blueye.protocol.types.message_formats import LatLongPosition

# Import position reset extension
from mission_planner_scripts.reset_drone_position import extend_ctrl_client

def main():
    # Command line arguments (minimal version for simplicity)
    if len(sys.argv) < 5:
        print("Usage: python goto_waypoint.py <waypoint_lat> <waypoint_lon> <depth> <drone_ip>")
        return 1
    
    waypoint_lat = float(sys.argv[1])
    waypoint_lon = float(sys.argv[2])
    depth = float(sys.argv[3])
    drone_ip = sys.argv[4] if len(sys.argv) >= 5 else "192.168.1.101"
    
    # Set up basic logging to console
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    logger = logging.getLogger("waypoint_mission")
    
    # Connect to the drone
    logger.info(f"Connecting to drone at {drone_ip}")
    drone = Drone(ip=drone_ip, auto_connect=True)
    logger.info(f"Connected to drone: {drone.serial_number}")
    
    # Take control if needed
    if not drone.in_control:
        drone.take_control()
        logger.info("Control of drone acquired")
    
    # Reset position to current position (for navigation reference)
    drone = extend_ctrl_client(drone)
    drone._ctrl_client.reset_position(waypoint_lat, waypoint_lon, 0.0)
    logger.info("Position reset to current coordinates")
    time.sleep(2)  # Brief pause
    
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
    
    # We should never reach here, but just in case
    return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)