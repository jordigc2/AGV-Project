#!/usr/bin/env python

import rospy
import envGraph as eg
import math as m
import numpy as np
from std_msgs.msg import Bool
from AGV5.msg import robot, goalPos
import time

path = 0

graph = eg.Graph()
newPath = True
alarm = False
nextNode = 0
compTaken = 0
prodList = graph.world.productsList

def getAlarmSignal(data):
	global alarm, newPath
	alarm = data.data
	print "QC Results:", alarm
	if alarm:
		newPath = True
	else:
		newPath = False
def test(data):
	global newPath, alarm, count, listPos, firstTime, path, nextNode, compTaken, prodList
	
	nextPos = goalPos()
	if newPath:#calculation of a new path, first time or when alarm is True
		print "robotPos: ", [data.x_pos, data.y_pos]
		graph.setRobotPosition(data.x_pos*100,data.y_pos*100)
		t0 = time.time()
		if not alarm:
			path = graph.calculatePath(alarm=True)
			print "time to create path: ", time.time()-t0
		else:
			graph.world.compRobot = rospy.get_param('componentsRobot')
			graph.world.compWareHouse = rospy.get_param('componentsWH')
			count = 0
			for comp in graph.world.compRobot:
				if comp != 0:
					graph.world.compAvRobot[count] = 1
				count += 1	
			print "A",prodList[0].prodID, prodList[0], prodList[0].numCompTaken		
			path = graph.alarmActivated()
			prodList = graph.world.productsList
			print "B",prodList[0].prodID, prodList[0], prodList[0].numCompTaken
			print "time to create alarm path: ", time.time()-t0


		#set 1st next position to go
		nextNode = path[0]
		path = np.delete(path,0)
		nextPos.x_pos = nextNode.x/100.0
		nextPos.y_pos = nextNode.y/100.0
		print "nextPos: ", [nextNode.x/100.0, nextNode.y/100.0]
		if not nextNode.id == 0 and not nextNode.id>3:
			nextPos.action = 0
		else:
			if nextNode.component != -1 and nextNode.component.returnComp:
				nextPos.action = 2
			else:
				nextPos.action = 1
			if nextNode.id == 0:
				nextPos.compID = -1
			else:
				prodList[0].inProgress = True
				prodList[0].startingTime = time.time()
				if prodList[0].numCompTaken == len(prodList[0].compList):
					prodList[1].numCompTaken += 1
					prodList[1].inProgress = True
					prodList[1].startingTime = time.time()
				else:
					if not nextNode.component.returnComp:
						prodList[0].numCompTaken += 1
				nextPos.compID = nextNode.component.compID

		print "nextNodeId:", nextNode.id
		listProd = []
		for i in range(4):
			listProd.append(prodList[i].prodID)
		rospy.set_param('prodList', listProd)
		rospy.set_param('nextNode', nextNode.id)
		rospy.set_param('prevProd', 0)

		pubPos.publish(nextPos)
		newPath = False
		
	prodList[0].inProgress = True
	dist = m.sqrt((data.x_pos-(nextNode.x/100.0))**2 + (data.y_pos-(nextNode.y/100.0))**2)
	#print "Distance to goal: ", dist
	#print "goal: ", [nextNode.x/100.0, nextNode.y/100.0]
	if dist < 0.1:
		if nextNode.id < 10 and nextNode.id >3 or nextNode.id == 0:
			if nextNode.id != 0:
				compTaken += 1
			rospy.sleep(1.5)
		nextNode = path[0]
		path = np.delete(path,0)
		nextPos.x_pos = nextNode.x/100.0
		nextPos.y_pos = nextNode.y/100.0
		if nextNode.id < 4 and nextNode.id != 0:
			nextPos.action = 0
		else:
			if nextNode.component != -1 and nextNode.component.returnComp:
				nextPos.action = 2
			else:
				nextPos.action = 1
			if nextNode.id != 0:
				prodList[0].inProgress = True
				if prodList[0].numCompTaken == len(prodList[0].compList):
					prodList[1].numCompTaken += 1
					prodList[1].inProgress = True
					prodList[1].startingTime = time.time()
				else:
					prodList[0].numCompTaken += 1	
				nextPos.compID = nextNode.component.compID
			else:
				nextPos.compID = -1
				
				print "compTaken:", prodList[0].numCompTaken,"neededComp:",len(prodList[0].compList)
				print "id:", nextNode.id
				if prodList[0].numCompTaken == len(prodList[0].compList) and nextNode.id == 0:
					print len(prodList), len(graph.world.productsList)
					print "All components taken for product", prodList[0].prodID
					prodList[0].numCompTaken = 0
					prodList[0].inProgress = False
					rospy.set_param('prodComp',prodList[0].compIDList)
					graph.world.prevProdDone = prodList[0]
					prodList = np.delete(prodList,0)
					prodList[0].inProgress = True
					graph.world.productsList = prodList
					print len(prodList), len(graph.world.productsList)
					pubAssembly.publish(True)
		print "nextPos: ", [nextNode.x/100.0, nextNode.y/100.0]
		pubPos.publish(nextPos)
		listProd = []
		for i in range(4):
			listProd.append(prodList[i].prodID)
		rospy.set_param('prodList', listProd)
		rospy.set_param('nextNode', nextNode.id)
		rospy.set_param('prevProd', graph.world.prevProdDone)
		

rospy.init_node('Routing')
pubPos = rospy.Publisher('goalPos', goalPos, queue_size=10)
pubAssembly = rospy.Publisher('productDone', Bool, queue_size=10)
rospy.Subscriber('robotPos', robot, test)
rospy.Subscriber('Alarm', Bool,getAlarmSignal)

if __name__ == '__main__':

	rospy.spin()
