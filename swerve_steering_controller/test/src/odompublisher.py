#!/usr/bin/python3

import sys
import rospy
from nav_msgs.msg import Odometry

def tf_callback(msg):
	global base_name, odom_publisher
	msg.header.frame_id="odom"
	odom_publisher.publish(msg)


rospy.init_node('odom_publisher', anonymous=True)
myargv = rospy.myargv(argv=sys.argv)
if len(myargv)!=2:
    base_name = "base_footprint"

else:
    base_name = myargv[1]
tempname1 = rospy.Subscriber("odomw", Odometry,tf_callback)
odom_publisher = rospy.Publisher('odom',Odometry,queue_size=5)

while not rospy.is_shutdown():
	continue










