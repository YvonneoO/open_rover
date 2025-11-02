#!/usr/bin/env python

import sys 
import rospy
from geometry_msgs.msg import Twist

#def datacollection():
#not subscribing anything


def direc():
    rospy.init_node('direc', anonymous = True)
    pub = rospy.Publisher('/cmd_verol/managed',Twist,queue_size = 10)
    rate =rospy.Rate(10) #10 Hertz
    while not rospy.is_shutdown():
	data = Twist() #object instance created 
	#data.header.stamp = rospy.Time.now()
	#data.header.frame_id = 'none'
	data.linear.x = 1.0
	data.linear.y = 0.0
	data.linear.z = 0.0
	data.angular.x = 0.0
	data.angular.y = 0.0
	data.angular.z = 0.0
	print(data)
	pub.publish(data)
	rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
	direc()
    except rospy.ROSInterruptException:
	print('What the hell')
	pass
