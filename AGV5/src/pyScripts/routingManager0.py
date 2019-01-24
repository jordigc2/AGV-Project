#!/usr/bin/env python

import rospy
import envGraph as eg
import math as m
from std_msgs.msg import Bool
from AGV5.msg import robot

listPos = [[0.6,0.4, 0],[1.4,1, 0], [1,1.6, 1]]
count = 0

def getAlarmSignal(data):
	print data
def test(data):
	global count
	global listPos
	global firstTime
	dist = m.sqrt((data.x_pos-listPos[count][0])**2 + (data.y_pos-listPos[count][1])**2)
	#print "Distance to goal: ", dist
	print listPos[count]
	if dist < 0.1:
		
		count += 1
		
		if count == 3:
			count = 0

		print "publishing goal pos"
		print listPos[count]
		goalPos = robot()
		goalPos.x_pos = listPos[count][0]
		goalPos.y_pos = listPos[count][1]
		goalPos.eul_pos = listPos[count][2]
		pubPos.publish(goalPos)
		print "published goal pos"

rospy.init_node('Routing', anonymous=True)
pubPos = rospy.Publisher('goalPos', robot, queue_size=10)
rospy.Subscriber('robotPos', robot, test)
rospy.Subscriber('Alarm', Bool,getAlarmSignal)

if __name__ == '__main__':
	try:
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			connections = pubPos.get_num_connections()
			if connections > 0:
				print connections
				goalPos = robot()
				goalPos.x_pos = listPos[0][0]
				goalPos.y_pos = listPos[0][1]
				goalPos.eul_pos = 0
				pubPos.publish(goalPos)
				break
			r.sleep()
	except rospy.ROSInterruptException, e:
		raise e


	rospy.spin()
