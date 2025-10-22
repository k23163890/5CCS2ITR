#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

#stores turtle's current position
currentPosition = Pose()

# Callback function to update turtle's current position
def poseMessage(msg):
    global currentPosition
    currentPosition = msg

#Mover function
def moveQuick(value, type, pub):
    if type == "linear": #Change linear if specified
        move = Twist()
        move.linear.x = value
        pub.publish(move)
        rospy.sleep(1)
    elif type == "angular": #change angular if specified
        move_cmd = Twist()
        move_cmd.angular.z = value
        pub.publish(move_cmd)
        rospy.sleep(1)
        


# Closed-loop function: move turtle to a starting corner
def reachCorner():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, poseMessage)
    rate = rospy.Rate(10)  # 10 Hz loop

    startingX = 2.0  # X coordinate of starting corner
    startingY = 2.0  # Y coordinate of starting corner

    while not rospy.is_shutdown():
        move = Twist()
        # Calculate distance and angle to target
        differenceX = startingX - currentPosition.x
        differenceY = startingY - currentPosition.y
        distance = math.hypot(differenceX, differenceY)
        angle = math.atan2(differenceY, differenceX) - currentPosition.theta

        # Stop if we are close enough
        if distance < 0.05:
            move.linear.x = 0
            move.angular.z = 0
            pub.publish(move)
            break

        # Move towards target
        move.linear.x = min(distance, 1.0)  # limit speed
        move.angular.z = angle
        pub.publish(move)
        rate.sleep()

    while abs(currentPosition.theta) > 0.05:
        move = Twist()
        move.angular.z = -0.5 * currentPosition.theta
        pub.publish(move)
        rate.sleep()

# Open-loop function: draw rectangle indefinitely
def draw_rectangle():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz loop
    width = 5.0   # rectangle width along x-axis
    height = 3.0  # rectangle height along y-axis

    while not rospy.is_shutdown():
        # Move forward along x-axis (longer side)
        moveQuick(width, "linear", pub)

        # Turn 90 degrees
        moveQuick(math.pi / 2, "angular", pub)

        # Move forward along y-axis (shorter side)
        moveQuick(height, "linear", pub)

        # Turn 90 degrees
        moveQuick(math.pi / 2, "angular", pub)

if __name__ == "__main__":
    rospy.init_node("turtle_rectangle_node")
    reachCorner()     # move to starting corner
    draw_rectangle()   # start drawing rectangle
