#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move_turtle():
	rospy.init_node('move_turtle', anonymous=False)
	# select topic
	# queue_size -> when not publishing, it store publishing in queue by n
	vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	
	# instantiation of data type
	vel_ref = Twist()

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		vel_ref.linear.x = 2
		vel_ref.angular.z = 2
		rospy.loginfo(vel_ref)
		vel_pub.publish(vel_ref)
		rate.sleep()

if __name__ == '__main__':
	try:
		move_turtle()
	except rospy.ROSInterruptException:
		pass
