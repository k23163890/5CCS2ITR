#!/urs/bin/env python3
import math

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus #Read the use of this in ITR week 5


rospy.init_node('go_to_point')

move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #name of action  = move_base, type = MoveBaseAction

move_base_client.wait_for_server()

goal = MoveBaseGoal() #Object of type move base goal


#Header stuff
goal.target_pose.header.stamp = rospy.get_rostime() #gets time

goal.target_pose.header.frame_id = 'map' # our frame is map


#Pose
goal.target_pose.pose.position.x = 1.95
goal.target_pose.pose.position.y = 8.66
goal.target_pose.pose.position.z = 0

#rotation 90 degrees downards
goal.target_pose.pose.orientation.x = 0  # sin pi / 2 = 0
goal.target_pose.pose.orientation.y = 0 # cos pi / 2
goal.target_pose.pose.orientation.z = math.sin(-math.pi/4) # Use Quaternions to understand this  - this dictates a 90 degree rotation
goal.target_pose.pose.orientation.w = math.cos(-math.pi/4)

move_base_client.send_goal(goal) # sending the goal message

move_base_client.wait_for_result()

status = move_base_client.get_state()

if(status == GoalStatus.SUCCEEDED):
    print("Worked")
else:
    print("failed")


# To run the robot:
    #rosrun <package> goto_point.py


