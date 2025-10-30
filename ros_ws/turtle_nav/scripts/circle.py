import rospy
from geometry_msgs.msg import Twist

rospy.init_node('circle_node', anonymous=True)
velocity_pub = rospy.Publisher('/turtle/cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():
    vel_mes = Twist()
    vel_mes.linear.x = 2
    vel_mes.angular


