import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    rospy.init_node('turtle_rectangle_node', anonymous=True)
    pub = rospy.Publisher('someSubscriber', Twist, queue_size=10)
    rate = rospy.Rate(1)

    mover = Twist()

    mover.linear.x = 2.0
    mover.linear.z = 0.0

    while not rospy.is_shutdown():
        pub.publish(mover)
        
        rospy.loginfo("Publishing velocity: linear=%f angular=%f" % (mover.linear.x, mover.angular.z))


if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass

