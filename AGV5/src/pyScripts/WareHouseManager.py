#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from AGV5.msg import robot, goalPos
import math as m
import time

TIME_ASSEMBLY = 15
TIME_QC = 5

whareHousePos = [1.25,0.7]
goingToWH = False
compLeaved = True
prodAssembled = False
compDelivered = False
timePassed = 0
timeAssembly = 0
prodComp = 0
qcResult = True

def publishAlarm(result):
	print "publishing QC results", result
	alarm = result
	alPub.publish(alarm)

def getGoal(data):
	global compLeaved, goingToWH, leavingComp, timeAssembly
	if data.compID == -1:
		goingToWH = True
		compLeaved = False
	elif compLeaved:
		goingToWH = False
def getProductStatus(data):
	global prodAssembled, prodComp, compDelivered
	prodComp = rospy.get_param('prodComp')
	compDelivered = True
	prodAssembled = False
	print "All the products", data.data
	

def isRobotClose(data):
	global whareHousePos, compLeaved, goingToWH, timeAssembly, prodAssembled, timePassed, compDelivered
	global TIME_ASSEMBLY, TIME_QC, qcResult
	if timeAssembly != 0:
		timePassed = time.time() - timeAssembly
		if timePassed >= TIME_ASSEMBLY and not prodAssembled:
			compWH = rospy.get_param('componentsWH')
			for compId in prodComp:
				compWH[compId-1] -= 1
			print "product assembled:", compWH
			rospy.set_param('componentsWH', compWH)
			timeAssembly = time.time()
			timePassed = 0
			prodAssembled = True
		if timePassed >= TIME_QC and prodAssembled:
			timeAssembly = 0
			prodAssembled = False
			timePassed = 0
			qcResult = True
			publishAlarm(qcResult)


	dist = m.sqrt((data.x_pos-whareHousePos[0])**2 + (data.y_pos-whareHousePos[1])**2)	
	if dist < 0.1 and goingToWH:
		leavingComp = True
		print "Leaving components to WH, all compDeliv:",compDelivered
		compRobot =  rospy.get_param('componentsRobot')
		#print compRobot
		compWH = rospy.get_param('componentsWH')
		count = 0
		print "compRobot:", compRobot
		for comp in compRobot:
			if comp != 0:
				compWH[comp-1] += 1
				compRobot[count] = 0
			count += 1
		if compDelivered:
			prodAssembled = False
			compDelivered = False
			print "Starting product assembly"
			timeAssembly = time.time()
		print "compWH:", compWH
		rospy.set_param('componentsWH', compWH)
		rospy.set_param('componentsRobot', compRobot)
		compLeaved = True
		goingToWH = False

rospy.set_param('componentsWH', [0]*6)
rospy.init_node('WareHouse')
rospy.Subscriber('robotPos', robot, isRobotClose)
rospy.Subscriber('goalPos', goalPos, getGoal)
rospy.Subscriber('productDone', Bool, getProductStatus)
alPub = rospy.Publisher('QC_int', Bool, queue_size=10)
def main():
	rospy.spin()

if __name__ == '__main__':
	main()
	
	
