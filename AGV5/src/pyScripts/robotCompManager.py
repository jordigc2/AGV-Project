#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from AGV5.msg import robot, goalPos
import math as m

compX = 0
compY = 0
idComp = 0
action = 0

newX = 0
newY = 0
newId = 0
newAct = 0
update = True
taken = True

def manageRobotComp(data):
	global compX, compY, idComp, action, taken
	compX = data.x_pos
	compY = data.y_pos
	idComp = data.compID
	action = data.action
	taken = False

def getRobotPos(data):
	global newX, newY, newId, newAct, update
	global compX, compY, idComp, action, taken
	if idComp > 0 and not taken:
		if update:
			print "newComp. X:", compX,"Y:", compY,"id:", idComp,"act:", action
			newX = compX
			newY = compY
			newId = idComp
			newAct = action
			update = False
		
		dist = m.sqrt((newX-(data.x_pos))**2 + (newY-(data.y_pos))**2)
		#print "dist:", dist
		if dist < 0.08:
			compRobot = rospy.get_param('componentsRobot')
			if action == 1:
				print "picking up component"
				if compRobot[0] == 0:
					compRobot[0] = int(newId)
				elif compRobot[1] == 0:
					compRobot[1] = int(newId)
			elif action == 2:
				print "leaving component"
				if compRobot[0] == newId:
					compRobot[0] = 0
				elif compRobot[1] == newId:
					compRobot[1] = 0
			print "compRobot:", compRobot
			update = True
			taken = True
			rospy.set_param('componentsRobot', compRobot)


rospy.set_param('componentsRobot', [0]*2)
rospy.init_node('Robot_manager')
rospy.Subscriber('goalPos', goalPos, manageRobotComp)
rospy.Subscriber('robotPos', robot, getRobotPos)
if __name__ == '__main__':
	rospy.spin()
