#!/usr/bin/env python3
"""
Direct SDK undocking script that connects to drone, performs undocking, then disconnects.
Similar to mission script approach to avoid connection conflicts.
"""

import time
import logging
import sys
import argparse
from blueye.sdk import Drone

def setup_logging():
    """Set up basic logging."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("undocking")

def connect_to_drone(drone_ip, timeout, logger):
    """Connect to the drone."""
    logger.info(f"Connecting to drone at {drone_ip}")
    try:
        drone = Drone(ip=drone_ip, auto_connect=True, timeout=timeout)
        
        logger.info(f"Connected to drone: {drone.serial_number}")
        logger.info(f"Drone software version: {drone.software_version}")
        
        if not drone.in_control:
            logger.info("Taking control of drone...")
            drone.take_control()
            logger.info("Control of drone acquired")
        
        return drone
        
    except Exception as e:
        logger.error(f"Failed to connect to drone: {str(e)}")
        return None

def perform_undocking_sequence(drone, reverse_duration=10, reverse_power=0.4, logger=None):
    """
    Perform undocking sequence using direct drone SDK commands.
    
    Args:
        drone: Connected Blueye drone object
        reverse_duration (int): How long to move backwards in seconds
        reverse_power (float): Power for backwards movement (0.1 to 1.0)
        logger: Logger object
    """
    try:
        # Enable station keeping (holds position and orientation)
        logger.info("Activating station keeping mode...")
        drone.motion.station_keeping_active = True
        time.sleep(3)  # Give time for station keeping to activate
        
        # Check if station keeping is active
        if drone.motion.station_keeping_active:
            logger.info("Station keeping is ACTIVE - drone will hold position")
        else:
            logger.warning("Station keeping failed to activate, using manual auto modes")
            # Fallback to manual auto modes
            drone.motion.auto_depth_active = True
            drone.motion.auto_heading_active = True
            time.sleep(2)
        
        # Hold position for a few seconds
        logger.info("Holding position for 5 seconds...")
        time.sleep(5)
        
        # Now move backwards while maintaining position control
        logger.info(f"Moving backwards at {reverse_power} power for {reverse_duration} seconds...")
        
        # Disable station keeping but keep auto modes for controlled backwards movement
        drone.motion.station_keeping_active = False
        drone.motion.auto_depth_active = True
        drone.motion.auto_heading_active = True
        time.sleep(1)  # Brief pause for mode change
        
        # Move backwards
        drone.motion.surge = -reverse_power
        
        # Monitor movement
        start_time = time.time()
        while time.time() - start_time < reverse_duration:
            try:
                current_depth = drone.depth
                logger.info(f"Moving backwards... Depth: {current_depth:.2f}m")
            except:
                logger.info("Moving backwards...")
            time.sleep(2)
        
        # Stop movement
        drone.motion.surge = 0.0
        logger.info("Backwards movement complete - drone stopped")
        
        # Re-enable station keeping to hold new position
        time.sleep(1)
        drone.motion.station_keeping_active = True
        logger.info("Station keeping re-activated - holding new position")
        
        # Hold the new position briefly
        time.sleep(3)
        
        logger.info("Undocking sequence completed successfully!")
        return True
        
    except Exception as e:
        logger.error(f"Error during undocking sequence: {str(e)}")
        return False

def disconnect_drone(drone, logger):
    """Disconnect from the drone safely."""
    if not drone or not drone.connected:
        return True
    
    logger.info("Disconnecting from drone")
    try:
        # Stop all movement first
        drone.motion.surge = 0.0
        drone.motion.sway = 0.0
        drone.motion.heave = 0.0
        drone.motion.yaw = 0.0
        
        # Disable auto modes
        drone.motion.station_keeping_active = False
        drone.motion.auto_depth_active = False
        drone.motion.auto_heading_active = False
        
        time.sleep(1)
        
        # Disconnect
        drone.disconnect()
        logger.info("Disconnected from drone")
        return True
        
    except Exception as e:
        logger.error(f"Error during disconnect: {str(e)}")
        return False

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='Perform undocking sequence using direct SDK')
    parser.add_argument('--drone-ip', type=str, default="192.168.1.101",
                        help='IP address of the drone')
    parser.add_argument('--timeout', type=int, default=30,
                        help='Connection timeout for the drone in seconds')
    parser.add_argument('--reverse-duration', type=int, default=10,
                        help='Duration to move backwards in seconds')
    parser.add_argument('--reverse-power', type=float, default=0.4,
                        help='Power for backwards movement (0.1 to 1.0)')
    
    args = parser.parse_args()
    
    # Validate reverse power
    if not 0.1 <= args.reverse_power <= 1.0:
        print("Error: reverse-power must be between 0.1 and 1.0")
        return 1
    
    # Set up logging
    logger = setup_logging()
    logger.info("Starting direct SDK undocking procedure")
    
    drone = None
    success = False
    
    try:
        # Connect to the drone
        drone = connect_to_drone(args.drone_ip, args.timeout, logger)
        if not drone:
            logger.error("Failed to connect to drone. Exiting.")
            return 1
        
        # Perform undocking sequence
        success = perform_undocking_sequence(
            drone, 
            args.reverse_duration, 
            args.reverse_power, 
            logger
        )
        
        logger.info(f"Undocking {'completed successfully' if success else 'failed'}")
        return 0 if success else 1
        
    except KeyboardInterrupt:
        logger.info("Undocking aborted by user")
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