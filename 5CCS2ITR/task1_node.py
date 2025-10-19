#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Global variable to store current turtle position
current_pose = Pose()

# Callback function to update turtle's current position
def pose_callback(msg):
    global current_pose
    current_pose = msg

# Closed-loop function: move turtle to a starting corner
def reach_corner():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)  # 10 Hz loop

    target_x = 2.0  # X coordinate of starting corner
    target_y = 2.0  # Y coordinate of starting corner

    while not rospy.is_shutdown():
        move_cmd = Twist()
        # Calculate distance and angle to target
        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx) - current_pose.theta

        # Stop if we are close enough
        if distance < 0.05:
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            pub.publish(move_cmd)
            break

        # Move towards target
        move_cmd.linear.x = min(distance, 1.0)  # limit speed
        move_cmd.angular.z = angle
        pub.publish(move_cmd)
        rate.sleep()

# Open-loop function: draw rectangle indefinitely
def draw_rectangle():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz loop
    width = 5.0   # rectangle width along x-axis
    height = 3.0  # rectangle height along y-axis

    while not rospy.is_shutdown():
        # Move forward along x-axis (longer side)
        move_cmd = Twist()
        move_cmd.linear.x = width
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Turn 90 degrees
        move_cmd = Twist()
        move_cmd.angular.z = math.pi / 2
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Move forward along y-axis (shorter side)
        move_cmd = Twist()
        move_cmd.linear.x = height
        pub.publish(move_cmd)
        rospy.sleep(1)

        # Turn 90 degrees
        move_cmd = Twist()
        move_cmd.angular.z = math.pi / 2
        pub.publish(move_cmd)
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("turtle_rectangle_node")
    reach_corner()     # move to starting corner
    draw_rectangle()   # start drawing rectangle
