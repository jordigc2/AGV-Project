#!/usr/bin/env python

import rospy 
from std_msgs.msg import Bool
from AGV5.msg import target


def cb(msg):
	global flag
	flag = True

flag = False

rospy.init_node('start_stop_node', anonymous=True)
sub = rospy.Subscriber("/some/topic", target, cb)
pub = rospy.Publisher("start/stop", Bool, queue_size=10)
while not flag == True:
	print "waiting.."
	pass

sub.unregister()


while not rospy.is_shutdown():

	
	if flag == True:
		raw_input("Ready for takeoff....")
		pub.publish(True)
		flag = not flag
	else: 
		raw_input("Press enter to stop .. (calma)")
		pub.publish(True)
		flag = not flag	 

