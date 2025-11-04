#!/usr/bin/env python

import rospy
import actionlib
from delivery_robot.actions import GoToLocationAction, LookAtAction
from delivery_robot.msg import Position

class DeliveryActionClient:
    """Client for the delivery robot mission."""

    def __init__(self):
        rospy.init_node('delivery_action_client')

        # 1. Action Client for GoToLocation
        self.goto_client = actionlib.SimpleActionClient('go_to_location', GoToLocationAction)
        rospy.loginfo("Waiting for GoToLocation action server...")
        self.goto_client.wait_for_server()
        rospy.loginfo("GoToLocation server found.")

        # 2. Action Client for LookAt
        self.lookat_client = actionlib.SimpleActionClient('look_at', LookAtAction)
        rospy.loginfo("Waiting for LookAt action server...")
        self.lookat_client.wait_for_server()
        rospy.loginfo("LookAt server found.")

        # Start the mission
        self.run_mission()

    def run_mission(self):
        rospy.loginfo("--- MISSION STARTING ---")

        # Location 1: Kitchen
        rospy.loginfo("\n*** Step 1: Navigating to 'kitchen' ***")
        goto_goal = GoToLocationAction.Goal(location_name="kitchen")
        self.goto_client.send_goal(goto_goal)
        self.goto_client.wait_for_result()
        if self.goto_client.get_result().success:
            rospy.loginfo("Step 1 Succeeded: Robot reached the kitchen.")
        else:
            rospy.logerr("Step 1 Failed: Could not reach the kitchen. Aborting mission.")
            return

        # Look at Person (Person is at x=5.0, y=5.0)
        rospy.loginfo("\n*** Step 2: Looking at person at (5.0, 5.0) ***")
        lookat_goal = LookAtAction.Goal(target_position=Position(x=5.0, y=5.0))
        self.lookat_client.send_goal(lookat_goal)
        self.lookat_client.wait_for_result()
        if self.lookat_client.get_result().success:
            rospy.loginfo("Step 2 Succeeded: Robot looked at the person.")
        else:
            rospy.logerr("Step 2 Failed: Could not look at the person. Continuing mission.")

        # Location 2: Living Room
        rospy.loginfo("\n*** Step 3: Navigating to 'living_room' ***")
        goto_goal = GoToLocationAction.Goal(location_name="living_room")
        self.goto_client.send_goal(goto_goal)
        self.goto_client.wait_for_result()
        if self.goto_client.get_result().success:
            rospy.loginfo("Step 3 Succeeded: Robot reached the living room.")
        else:
            rospy.logerr("Step 3 Failed: Could not reach the living room.")

        rospy.loginfo("\n--- MISSION COMPLETE ---")

if __name__ == '__main__':
    try:
        DeliveryActionClient()
    except rospy.ROSInterruptException:
        pass