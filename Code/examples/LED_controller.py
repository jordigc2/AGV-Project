#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

TURN_ON = 1
TURN_OFF = 0

def talker():
	pub = rospy.Publisher('LEDaction', Int16) #create topic
	rospy.init_node('LEDmaster', anonymous = True) #create Publisher node
	rate = rospy.Rate(10)

	LED = TURN_OFF
	while not rospy.is_shutdown():
		if(LED == TURN_OFF):
			LED = TURN_ON
		else:
			LED = TURN_OFF
		rospy.loginfo(LED) #debugging tool to see variables
		pub.publish(LED) #publish LED to the topic
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
