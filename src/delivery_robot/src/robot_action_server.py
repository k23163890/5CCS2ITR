#!/usr/bin/env python

import rospy
import actionlib
import math

from delivery_robot.actions import GoToLocationAction, LookAtAction
from delivery_robot.srv import GetLocationCoordinates, GetLocationCoordinatesRequest
from delivery_robot.msg import Position

class RobotActionServer:
    """Implements the GoToLocation and LookAt action servers."""

    def __init__(self):
        rospy.init_node('robot_action_server')

        # 1. Action Server: GoToLocation
        self.goto_server = actionlib.SimpleActionServer(
            'go_to_location', GoToLocationAction, 
            execute_cb=self.execute_goto, auto_start=False
        )
        self.goto_server.start()

        # 2. Action Server: LookAt
        self.lookat_server = actionlib.SimpleActionServer(
            'look_at', LookAtAction, 
            execute_cb=self.execute_lookat, auto_start=False
        )
        self.lookat_server.start()

        # 3. Service Client: GetLocationCoordinates
        rospy.wait_for_service('get_location_coords')
        self.get_coords_service = rospy.ServiceProxy('get_location_coords', GetLocationCoordinates)
        
        # Simulated Robot State (start at origin, facing 0 rad)
        self.current_pos = Position(x=0.0, y=0.0)
        self.current_yaw = 0.0 # Yaw angle in radians

        rospy.loginfo("Robot Action Server ready.")
        rospy.spin()

    def execute_goto(self, goal):
        """Handles the GoToLocation action."""
        rospy.loginfo(f"GoToLocation: Received goal to navigate to '{goal.location_name}'.")
        feedback = GoToLocationAction.Goal()
        rate = rospy.Rate(1)

        # 1. Use Service to get coordinates
        try:
            req = GetLocationCoordinatesRequest(location_name=goal.location_name)
            res = self.get_coords_service(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.goto_server.set_aborted()
            return

        if not res.success:
            rospy.logerr(f"Could not find coordinates for location: {goal.location_name}")
            self.goto_server.set_aborted()
            return

        target_pos = res.coordinates
        rospy.loginfo(f"Target coordinates: ({target_pos.x}, {target_pos.y})")

        # 2. Simulated Navigation Loop
        distance_threshold = 0.5
        while not rospy.is_shutdown():
            if self.goto_server.is_preempt_requested():
                rospy.loginfo('GoToLocation Preempted')
                self.goto_server.set_preempted()
                return
            
            # --- Simulated Movement ---
            dx = target_pos.x - self.current_pos.x
            dy = target_pos.y - self.current_pos.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance < distance_threshold:
                # Target reached
                self.current_pos = target_pos
                rospy.loginfo('GoToLocation Succeeded: Target reached.')
                self.goto_server.set_succeeded(GoToLocationAction.Result(success=True))
                return

            # Simulate step towards target (e.g., 20% of remaining distance per cycle)
            step_factor = 0.2
            self.current_pos.x += dx * step_factor
            self.current_pos.y += dy * step_factor
            
            # Publish Feedback
            feedback.current_position = self.current_pos
            self.goto_server.publish_feedback(feedback)
            rate.sleep()

    def execute_lookat(self, goal):
        """Handles the LookAt action."""
        rospy.loginfo(f"LookAt: Received goal to look at ({goal.target_position.x}, {goal.target_position.y}).")
        feedback = LookAtAction.Goal()
        rate = rospy.Rate(5)
        
        # 1. Calculate desired yaw to target
        dx = goal.target_position.x - self.current_pos.x
        dy = goal.target_position.y - self.current_pos.y
        # atan2 gives angle in radians from -pi to pi
        desired_yaw = math.atan2(dy, dx)
        
        angle_threshold = 0.1 # radians

        # 2. Simulated Orientation Loop
        while not rospy.is_shutdown():
            if self.lookat_server.is_preempt_requested():
                rospy.loginfo('LookAt Preempted')
                self.lookat_server.set_preempted()
                return
            
            # Calculate shortest angular distance (error)
            error = desired_yaw - self.current_yaw
            # Normalize error to the range [-pi, pi]
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < angle_threshold:
                # Target orientation reached
                self.current_yaw = desired_yaw
                rospy.loginfo(f"LookAt Succeeded. Final Yaw: {self.current_yaw:.2f} rad.")
                self.lookat_server.set_succeeded(LookAtAction.Result(success=True))
                return

            # --- Simulated Rotation ---
            rotation_speed = 0.5 # radians per second
            if error > 0:
                self.current_yaw += rotation_speed / 5.0 # Rotate towards target
            else:
                self.current_yaw -= rotation_speed / 5.0
            
            # Keep current_yaw within [-pi, pi]
            self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

            # Publish Feedback
            feedback.orientation_error = abs(error)
            self.lookat_server.publish_feedback(feedback)
            rate.sleep()

if __name__ == '__main__':
    try:
        RobotActionServer()
    except rospy.ROSInterruptException:
        pass