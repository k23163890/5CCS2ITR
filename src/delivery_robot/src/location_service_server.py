#!/usr/bin/env python

import rospy
from delivery_robot.srv import GetLocationCoordinates, GetLocationCoordinatesResponse
from delivery_robot.msg import Position

class LocationServiceServer:
    """Implements the service for mapping location names to coordinates."""

    def __init__(self):
        rospy.init_node('location_service_server')
        # Define locations and coordinates
        self.locations = {
            "kitchen": Position(x=1.0, y=5.0),
            "living_room": Position(x=10.0, y=2.0)
        }
        
        rospy.Service('get_location_coords', GetLocationCoordinates, self.handle_get_location_coords)
        rospy.loginfo("Location Service Server ready.")
        rospy.spin()

    def handle_get_location_coords(self, req):
        """Service callback: takes location_name, returns Position and success status."""
        loc_name = req.location_name
        
        if loc_name in self.locations:
            rospy.loginfo(f"Request for '{loc_name}' received. Returning coordinates.")
            return GetLocationCoordinatesResponse(
                coordinates=self.locations[loc_name],
                success=True
            )
        else:
            rospy.logwarn(f"Location '{loc_name}' not found.")
            # Return empty position and failure
            return GetLocationCoordinatesResponse(
                coordinates=Position(x=0.0, y=0.0),
                success=False
            )

if __name__ == '__main__':
    try:
        LocationServiceServer()
    except rospy.ROSInterruptException:
        pass