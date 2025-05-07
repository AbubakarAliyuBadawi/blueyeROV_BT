"""
Extension module for Blueye SDK to add reset position functionality.

This module adds the ability to programmatically reset the drone's position
to a specified GPS coordinate, similar to the "Reset to POI" function in the app.
"""

import blueye.protocol as bp
from blueye.sdk import Drone


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