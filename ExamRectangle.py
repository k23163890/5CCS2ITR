#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import math
import tf.transformations as tft

class SimplePose:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

pose = SimplePose()

def pose_cb(msg: Odometry):
    global pose
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = tft.euler_from_quaternion(orientation_list)
    pose.theta = yaw

def doMaths(corner_x, corner_y):
    dx = corner_x - pose.x
    dy = corner_y - pose.y
    d = math.hypot(dx, dy)
    ang = math.atan2(dy, dx)
    err = math.atan2(math.sin(ang - pose.theta), math.cos(ang - pose.theta))
    return d, err

def move_forward(pub, rate, speed, dist):
    travelled = 0.0
    duration = dist / speed
    start_time = rospy.get_time()

    while not rospy.is_shutdown() and (rospy.get_time() - start_time) < duration:
        cmd = Twist()
        cmd.linear.x = speed
        pub.publish(cmd)
        rate.sleep()
    
    pub.publish(Twist())
    rospy.sleep(0.1)


def turn_left(pub, rate, turn_speed, angle):
    turned = 0.0
    duration = abs(angle) / turn_speed
    start_time = rospy.get_time()

    while not rospy.is_shutdown() and (rospy.get_time() - start_time) < duration:
        cmd = Twist()
        cmd.angular.z = turn_speed
        pub.publish(cmd)
        rate.sleep()
        
    pub.publish(Twist())
    rospy.sleep(0.1)


def reach_corner(pub, rate):
    while not rospy.is_shutdown():
        err = math.atan2(math.sin(0 - pose.theta), math.cos(0 - pose.theta))
        if abs(err) < 0.01:
            break
        cmd = Twist()
        cmd.angular.z = 4 * err
        pub.publish(cmd)
        rate.sleep()
    pub.publish(Twist())
    rospy.sleep(0.1)


def closed_loop_corner():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    corner_x, corner_y = 3.0, 2.0
    rospy.loginfo("Moving to closed-loop corner (3.0, 2.0)")
    while not rospy.is_shutdown():
        vals = doMaths(corner_x,corner_y)
        d, err = vals[0], vals[1]
        
        if d < 0.01:
            break
            
        cmd = Twist()
        cmd.linear.x = min(1.0, 2 * d)
        cmd.angular.z = 6 * err
        pub.publish(cmd)
        rate.sleep()
        
    pub.publish(Twist())
    rospy.sleep(0.5)
    
    rospy.loginfo("Arrived at corner. Aligning to zero angle.")
    reach_corner(pub, rate)

def open_loop_rectangle():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(40)
    L = 5.0
    H = 3.0
    speed = 0.5
    turn_speed = 0.5
    
    rospy.loginfo("Starting open-loop rectangle drawing.")

    for i in range(4):
        distance = L if i % 2 == 0 else H
        rospy.loginfo(f"Moving forward {distance}m")
        move_forward(pub, rate, speed, distance)
        
        if i < 3:
            rospy.loginfo("Turning 90 degrees left")
            turn_left(pub, rate, turn_speed, math.pi / 2)


if __name__ == "__main__":
    rospy.init_node("robot_path_node")
    
    rospy.Subscriber("/odom", Odometry, pose_cb)
    
    rospy.sleep(1)

    try:
        closed_loop_corner()
        open_loop_rectangle()
    except rospy.ROSInterruptException:
        pass
    finally:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub.publish(Twist())
        rospy.loginfo("Robot movement sequence finished/interrupted.")
