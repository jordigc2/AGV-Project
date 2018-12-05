import sys
import world as wd
import math
import time
import bisect
import numpy as np
import random
from operator import attrgetter
import copy
import itertools



class Node:
	def __init__(self,x, y, _id, component = -1):
		"""
			Param: x, y position of the element, _id of the element, from 0-3 objects in the world 
			and from 4-9 components and component is the Component object
			Return: None
			Description: Initializes a node according to the object that represents.
		"""
		self.x = x
		self.y = y
		self.id = _id
		self.neigh = []
		self.prevNeigh = 0
		self.robotPrevNeigh = 0
		self.dijkstraDistance = sys.maxsize #set a huge number to the distances
		self.robotDistance = sys.maxsize #set a huge number to the distances
		self.sortDistance = sys.maxsize
		self.visited = False
		self.component = component

	#used when inserting nodes to a list, to have the list sorted by distance
	def __lt__(self, other):
		"""
		 Param: Object Node other
		 Return: bool
		 Description: Used to have a list of Nodes sorted by the dijkstraDistance attribute using 
		 the library bisect
		"""
		return(self.sortDistance < other.sortDistance)


class Graph:
	def __init__(self):
		"""
		Param: None
		Return: None
		Description: Initialize the mapping of the real map into a graph. First the world objects
		are inserted to the graph and then the nodes corresponding to the 6 components.
		"""

		self.listOfNodes = [] #0  WH, 1-3 wallnodes and 4-9 components nodes
		self.robotNode = Node(0, 0, 10)

		self.world = wd.World()

		count = 0
		#creation of the nodes of the graph
		for obj in self.world.objectsPos:
			node = Node(obj[0],obj[1], count)
			self.listOfNodes.append(node)
			count += 1

		for comp in self.world.availableComp:
			node = Node(comp.comPos[0], comp.comPos[1], comp.compID+count-1, comp)
			self.listOfNodes.append(node)

		self.setNeighbours()
		self.robotNode.neigh = self.listOfNodes[1:]
		self.calculateDijkstraDistances(0,0)

	def setRobotPosition(self, x, y):
		self.robotNode.x = x
		self.robotNode.y = y

	def setNeighbours(self):
		"""
			Param: None
			Return: None
			Description: Set the nodes neighbour in order to be able to move around the graph. The 
			neighbours is a list of nodes.
		"""

		count = 0
		for node in self.listOfNodes:
			if count == 0:
				#set warehouse neighbours
				for aux in self.listOfNodes[1:self.world.numObjects]:
					node.neigh.append(aux)
			elif count >= 1 and count < self.world.numObjects:
				#set wall node neighbours
				node.neigh.append(self.listOfNodes[0])
				for aux in self.listOfNodes[self.world.numObjects:len(self.listOfNodes)]:
					node.neigh.append(aux)
			else:
				#set components nodes neighbours
				for aux in self.listOfNodes[1:self.world.numObjects]:
					node.neigh.append(aux)

				for aux in self.listOfNodes[self.world.numObjects:len(self.listOfNodes)]:
					node.neigh.append(aux)
			count += 1

	def dijkstraDistancesNoQueue(self,startingNodeID, alarm=False):
		for node in self.listOfNodes:
			node.dijkstraDistance = sys.maxsize
			node.visited = False
			node.robotDistance = sys.maxsize

		actNode = self.listOfNodes[startingNodeID]
		numNodes = len(self.listOfNodes)
		completed = False
		if not alarm:
			actNode.dijkstraDistance = 0
			actNode.visited = True
			while not completed:
				for node in actNode.neigh:
					if not node.visited:
						dist = actNode.dijkstraDistance + math.sqrt((node.x-actNode.x)**2+(node.y-actNode.y)**2)
						if dist < node.dijkstraDistance:
							node.dijkstraDistance = dist
							node.prevNeigh = actNode
				minDist = sys.maxsize
				for auxnodes in self.listOfNodes:
					if auxnodes.dijkstraDistance < minDist and not auxnodes.visited:
						actNode = auxnodes
						minDist = auxnodes.dijkstraDistance
				actNode.visited = True
				numNodes -= 1

				if numNodes == 0:
					completed = True

	def calculateDijkstraDistances(self, startingNodeID, initDistance, alarm = False):
		"""
			Param: startingNodeID, this node will have dijkstraDistance = 0; initDistance is to know
			how far is the robot from the starting node.
			Return: None
			Description: Run over all the nodes of the graph using a priority queue in order to go 
			faster and calculate the shortest distance from the starting node.
		"""
		if not alarm:
			startNode = self.listOfNodes[startingNodeID]
			startNode.dijkstraDistance = initDistance
			for node in self.listOfNodes:
				node.visited = False
				node.dijkstraDistance = sys.maxsize
			priorityQueue = [startNode]

			while len(priorityQueue) > 0:
				actualNode = priorityQueue.pop(0)
				if not actualNode.visited:

					for node in actualNode.neigh:
						dist = actualNode.dijkstraDistance + math.sqrt((node.x-actualNode.x)**2+(node.y-actualNode.y)**2)
						if dist < node.dijkstraDistance:
							node.dijkstraDistance = dist
							node.sortDistance = dist
							node.prevNeigh = actualNode
							bisect.insort_left(priorityQueue, node)

					actualNode.visited = True
		else:
			startNode = self.robotNode
			startNode.robotDistance = initDistance
			priorityQueue = [startNode]
			for node in self.listOfNodes:
				node.visited = False
				node.robotDistance = sys.maxsize	

			while len(priorityQueue) > 0:
				actualNode = priorityQueue.pop(0)
				if not actualNode.visited:

					for node in actualNode.neigh:
						if node.id > 3 and node.id < 10 and actualNode.y >= 164 or node.id<=3:							
							dist = actualNode.robotDistance + math.sqrt((node.x-actualNode.x)**2+(node.y-actualNode.y)**2)
							if dist < node.robotDistance:
								node.robotDistance = dist
								node.robotPrevNeigh = actualNode
								node.sortDistance = dist
								bisect.insort_left(priorityQueue, node)

					actualNode.visited = True

	def sortCompDist(self, product, alarm):
		"""
			Param: product, to be able to get the list of components needed.
			Return: compNodes, is a list of sorted nodes corresponing to a component.
			Description: Using the bisect libary, a list of nodes is created but the nodes are
			placed sorted inside the list.
		"""

		compNodes = []
		sortedCompList = [0]*len(product.compList)
		for comp in product.compList:				
			node = copy.copy(self.listOfNodes[comp.compID+self.world.numObjects-1])
			if not alarm:
				node.sortDistance = node.dijkstraDistance
			else:
				node.sortDistance = node.robotDistance
			node.component = comp
			bisect.insort_left(compNodes, node)

		return compNodes

	def gotToWH(self, node, direction, alarm=False):
		"""
			Param: node, is the actual node that the robot is and direction, to know if the path 
			goes from the WH to the node or the other way around.
			Return: subPath
			Description: Generates a subPath to be able to go or return from a node from the WH.
		"""
		subPath = np.array([node])
		if not alarm or alarm == 2:
			idGoal = 0
		else:
			idGoal = 10
		while node.id != idGoal:#insert to the subPath until reaching to the desired node
			if not alarm or alarm == 2:
				node = node.prevNeigh
			else:
				node = node.robotPrevNeigh
			if direction == 0:#from actual position to node
				subPath = np.insert(subPath,0, node)
			else:#from node to WH
				subPath = np.insert(subPath,len(subPath), node)

		return subPath

	def getClosestComp(self, nodeList, actNode):
		"""
			Param: nodeList, components of a product and actNode, to know where the robot is at each
			moment.
			Return: Node
			Description: It gives the node of the closes component from another component Node that 
			has not been collected yet.
		"""
		minDis = sys.maxsize
		closestComp = 0
		for aux in nodeList:
			if not aux.component.collected:
				dist = math.sqrt((aux.x-actNode.x)**2+(aux.y-actNode.y)**2)
				if dist < minDis:
					minDis = dist
					closestComp = aux
		return closestComp


	def generatePath(self, product, compTaken, actNode, alarm):
		"""
			Param: product, used to get the components needed, compTaken, how many components is 
			planned to have the robot at each moment, actNode, to know the path to the next object.
			Return: subPath and compTaken
			Description: It creates the path needed to get all the components of a product. It is 
			checked how many comp is planned to have the robot at each moment. Because, after 
			getting all the components of a product, there might be an empty gap, which will be 
			used with one of the component of the following component.
		"""

		nodesComp = self.sortCompDist(product, alarm)
		"""print("Components Sorted")
		for node in nodesComp:
			print(node.id, node.sortDistance, node.component)"""

		path = np.array([], dtype="object")
		pathCreated = False
		FROM_HOME = 0
		FROM_NODE = 1
		
		while not pathCreated:
			#print("compTaken:", compTaken, "al:",alarm)
			if compTaken == 1 or alarm == 3 or alarm == 4:
				if alarm == 3:
					actNode = self.robotNode
					#print("actNode:",actNode.id)
					for compID in self.world.compRobot:#add components to return into the  path
						if compID != 0:
							node = self.listOfNodes[compID+self.world.numObjects-1]
							if node.component.returnComp:
								path = np.insert(path,len(path),node)
								actNode = node

					closestNode = self.getClosestComp(nodesComp, actNode)
					if closestNode != 0:# if zero, there are no left components to take of the product
						path = np.insert(path,len(path),closestNode)
						closestNode.component.collected = True
						compTaken = 0
					actNode = closestNode

				elif alarm == 4:
					node1 = self.listOfNodes[self.world.compRobot[0]+self.world.numObjects-1]
					node2 = 0
					if self.world.compRobot[1] != 0:
						node2 = self.listOfNodes[self.world.compRobot[1]+self.world.numObjects-1]
					if node2 != 0:
						#May need a second look in a future
						if node1.robotDistance <= node2.robotDistance:
							path = np.array([node1], dtype="object")
							actNode = node1
							compTaken = 1
						else:
							path = np.array([node2], dtype="object")
							actNode = node2
							compTaken = 1
					else:
						self.world.compAvRobot[0] = 0
						compTaken = 1
						actNode = self.robotNode

				alarm = False
				if compTaken == 1:
					closestNode = self.getClosestComp(nodesComp, actNode)
					if closestNode != 0:# if zero, there are no left components to take of the product
						path = np.insert(path,len(path),self.gotToWH(closestNode, FROM_NODE))
						closestNode.component.collected = True
						compTaken = 0
				elif compTaken == 2:
					compTaken = 0
			else:
				for node in nodesComp:
					if not node.component.collected:
						#print("FirstComonent To take:", node.id)
						node.component.collected = True
						subPath = self.gotToWH(node, FROM_HOME, alarm)
						if(len(path) > 0):
							actNode = path[len(path)-1]

						if(subPath[0].id == actNode.id):
							subPath = np.delete(subPath,0)
						path = np.insert(path,len(path),subPath)
						"""print("path:")
						for node in path:
							print(node.id)"""
						compTaken = 1
						count2 = 0
						actNode = node
						closestNode = self.getClosestComp(nodesComp, actNode)
						if alarm != 0:
							alarm = False
						#if(closestNode != 0):
						#	print("Closest:",closestNode.id)
						if closestNode != 0:
							path = np.insert(path,len(path),self.gotToWH(closestNode, FROM_NODE))
							closestNode.component.collected = True
							compTaken = 0
				pathCreated = True

		return path,compTaken

	def calculatePath(self, alarm = False, product = -1):
		"""
			Param: product, to calculate the path of  single product if is different to -1. alarm, 
			to know if there is any alarm or not.
			Return: path, path at time zero.
			Description: It goes trough all the products and it generates the path of how to reach 
			to evry single component in order to get the products assembled.
		"""

		path = np.array([], dtype="object") #numpy array of nodes
		optimalDistance = sys.maxsize
		currentDistance = 0
		compTaken = 0
		actNode = self.listOfNodes[1]
		count = 0
		al = 1 
		self.calculateDijkstraDistances(0, 0, alarm=al)
		if product == -1:
			listOfProducts  = self.world.productsList
		else:
			listOfProducts = [product]

		for product in listOfProducts:
			#print("product:", product.prodID, product.compList)
			if not alarm:
				product.uncollectComp()
				if product.inProgress:
					timesAsked = 1
					for comp in product.compList:
						if self.world.compWareHouse[comp.compID-1] > 0:
							if not comp.compID in self.world.prevProdDone.compIDList:
								comp.collected = True
							else:
								if self.world.compWareHouse[comp.compID-1] > timesAsked:
									comp.collected = True
									timesAsked += 1
					product.numCompTaken = compTaken
					product.inProgress = False
			if sum(self.world.compAvRobot)>0:
				for comp in product.compList:		
					if comp.compID in self.world.compRobot:
						indx = self.world.compRobot.index(comp.compID)
						if self.world.compAvRobot[indx] == 1:
							comp.collected = True
							compTaken += 1
							self.world.compAvRobot[indx] -= 1
			count += 1

			if(len(path)>0):
				actNode = path[len(path)-1]
			prodPath,compTaken = self.generatePath(product, compTaken, actNode, alarm)
			#print("compTaken", compTaken, product.prodID, alarm)
			#print(" path:",prodPath)
			if not alarm:
				path = np.insert(path,len(path), prodPath)
			else:
				path = np.insert(path,0, prodPath)

			alarm = False

		if(compTaken == 1 and product == -1):
			lastComp = path[len(path)-1]
			path = np.delete(path,len(path)-1)
			path = np.insert(path,len(path), self.gotToWH(lastComp,1)) 

		if path[0].id == 10:
			path = np.delete(path,0)

		return path

	def alarmActivated(self, mode = 0):
		"""
			Param: None
			Return: updatedPath
			Description: This function runs when a product is defected and updates the path in order
			to get the components to redo the product. This function will read from the ROS topics 
			of the components in the warehous, the components in the robot and the position of the
			robot.
		"""
		#robotPos = self.getAGVPos()
		#robotPos = [x,y+10]
		self.world.compRetRobot = [0]*2

		print("Robot is at:", [self.robotNode.x,self.robotNode.y])

		self.world.prevProdDone.numCompTaken = 0
		prodDefect = self.world.prevProdDone
		print(prodDefect)
		comList = prodDefect.compList
		prodDefect.uncollectComp()
		count = 0
		if sum(self.world.compRobot) == 0: 
			#robot has no components
			print("robot has no components")
			print "compWH", self.world.compWareHouse
			for comp in comList:
				if self.world.compWareHouse[comp.compID-1] > 0:
					print "already in the WH", comp.compID
					comp.collected = True
					prodDefect.numCompTaken += 1

			self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
			if mode == 0:
				path = self.calculatePath(alarm=1)
			else:
				path = self.calculatePath(alarm=1, product=prodDefect)
		else:
			print("Robot has components:", self.world.compRobot)
			if self.robotNode.y <= 164: #robot in sector1
				print("Robot in Sector1")
				for comp in comList:
					if comp.compID in self.world.compRobot or self.world.compWareHouse[comp.compID-1] > 0:
						prodDefect.numCompTaken += 1
						comp.collected = True
						if comp.compID in self.world.compRobot:
							self.world.compAvRobot[self.world.compRobot.index(comp.compID)] -= 1
				self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
				if mode == 0:
					path = self.calculatePath(alarm=1)
				else:
					path = self.calculatePath(alarm=1, product=prodDefect)
			else:
				print("Robot in Sector2")
				compRobNeed = 0
				indxSame = -1
				for comp in comList:
					if comp.compID in self.world.compRobot:
						aux = self.world.compRobot.index(comp.compID)
						if aux != indxSame:
							comp.collected = True
							prodDefect.numCompTaken += 1
							indxSame = aux
							compRobNeed += 1
					elif self.world.compWareHouse[comp.compID-1] > 0:
						comp.collected = True
						prodDefect.numCompTaken += 1

				if compRobNeed == 2: #the robot has two products needed to assemble the product
					print("Robot has 2 comp needed")
					self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
					#print(self.world.productsList)
					if mode == 0:
						path = self.calculatePath(alarm=2)
					else:
						path = self.calculatePath(alarm=2, product=prodDefect)
				elif compRobNeed == 1: #the robot has 1 comp not needed to assemble the product.
					#WORKING*
					print("Robot has 1 comp needed")
					if len(self.world.productsList[0].compList) % 2 == 0 or\
																	 sum(self.world.compAvRobot)==2:

						self.world.compAvRobot = [0,0]
					self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
					for compID in self.world.compRobot:
						node = self.listOfNodes[compID+self.world.numObjects-1]
						#print("nodeID:", node.id, "compID:",compID)
						if compID != 0:
							if not compID in prodDefect.compIDList:
								node.component.returnComp = True
								self.world.compRetRobot[0] = compID
							else:
								node.component.returnComp = False
					if mode == 0:
						path = self.calculatePath(alarm=3)
					else:
						path = self.calculatePath(alarm=3, product=prodDefect)
				else: #all the comp that has the robot are not needed now
					print("Robot has 0 comp needed")
					"""if len(self.world.productsList[0].compList) % 2 == 0 or\
																	 sum(self.world.compAvRobot)==2:"""

					#self.world.compAvRobot = [0,0]
					print("compAv:", self.world.compAvRobot)
					self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
					for compID in self.world.compRobot:
						node = self.listOfNodes[compID+self.world.numObjects-1]
						#print("nodeID:", node.id, "compID:",compID)
						if compID != 0:
							if not compID in prodDefect.compIDList:
								node.component.returnComp = True
								if self.world.compRetRobot[0]==0:
									self.world.compRetRobot[0] = compID
								else:
									self.world.compRetRobot[1] = compID
							else:
								node.component.returnComp = False

					if mode == 0:
						path = self.calculatePath(alarm=4)
					else:
						path = self.calculatePath(alarm=4, product=prodDefect)

		print "Components Taken", self.world.productsList[0]
		if path[0].id == 10:
			path = np.delete(path,0)
		return path


	def getOptimalPath(self, products):
		"""
			Param: products. A list of 2 products, the next one and two after.
			Return: path. To collect all the components of product1
			Description: Calculate the path to collect a product taking into account the followinf 
			product in order to minimize the distance
		"""

		path = np.array([], dtype="object") #numpy array of nodes
		compList1 = []
		compList2 = []
		count = 0
		self.calculateDijkstraDistances(0,0, alarm = True)
		compTaken = 0
		for prod in products:
			prod.uncollectComp()
			if prod.inProgress:
				timesAsked = 1
				for comp in prod.compList:
					if self.world.compWareHouse[comp.compID-1] > 0:
						if not comp.compID in self.world.prevProdDone.compIDList:
							comp.collected = True
						else:
							if self.world.compWareHouse[comp.compID-1] > timesAsked:
								comp.collected = True
								timesAsked += 1
				prod.numCompTaken = compTaken
				prod.inProgress = False

			count = 0
			for compID in prod.compIDList:
				if not prod.compList[count].collected:
					if count == 0:
						compList1.append(self.listOfNodes[compID+self.world.numObjects-1])
					else:
						compList2.append(self.listOfNodes[compID+self.world.numObjects-1])
			count += 1

		compProd1Prem = list(itertools.permutations(products[0].compList))
		permProd1 = list(itertools.permutations(compList1))
		permProd2 = list(itertools.permutations(compList2))
		if sum(self.world.compRobot) == 0: 
			#robot has no components
			compRobotTaken = 0
		else:
			compRobotTaken = 1

		i = 0
		optDistance = sys.maxsize
		optPerm = [0,0]
		for perm1 in permProd1:
			j = 0
			dist1 = 0
			count = 0
			prevNode = 0
			numTrips = 0
			actualWHstate = copy.copy(self.world.compWareHouse)
			for node in perm1:
				if actualWHstate[node.component.compID-1] == 3 and numTrips == 0:
					dist1 = sys.maxsize
				else:
					actualWHstate[node.component.compID-1] += 1
					if count%2==0: #go from robot position or from wh
						if count == 0:
							if compRobotTaken == 0:
								dist1 += node.robotDistance
							else:
								dist1 += math.sqrt((self.robotNode.x-node.x)**2+\
									(self.robotNode.y-node.y)**2)
						else:
							dist1 += node.dijkstraDistance
					else:
						dist1 += math.sqrt((prevNode.x-node.x)**2+(prevNode.y-node.y)**2)
						numTrips += 1
				count += 1

				prevNode = node

			if len(perm1)%2==0:
				compTaken = compRobotTaken - 0
			else:
				comTaken = 1-compRobotTaken

			prevCompTaken = compTaken
			if dist1 < optDistance:
				for perm2 in permProd2:
					dist2 = 0
					compTaken = prevCompTaken
					for node in perm2:
						if compTaken == 0:
							dist2 += node.dijkstraDistance
							compTaken += 1
						else:
							dist2 += math.sqrt((prevNode.x-node.x)**2+(prevNode.y-node.y)**2)
							compTaken = 0

						prevNode = node
					totalDistance = dist1+dist2
					if totalDistance < optDistance:
						optDistance = totalDistance
						optPerm = [i,j]
						print "optPerm:", optPerm, " distance:", optDistance
					j += 1
			i += 1

		for node in permProd1[optPerm[0]]:
			print node.id
		print "____"
		if compRobotTaken == 0:
			count = 0
			compTaken = 0
			for node in permProd1[optPerm[0]]:
				if compTaken == 0:
					if count == 0:
						path = np.insert(path,len(path),self.gotToWH(node, 0, 1))
						path = np.delete(path,0	)
					else:
						subPath = self.gotToWH(node, 0)
						if(len(path) > 0):
							actNode = path[len(path)-1]

						if(subPath[0].id == actNode.id):
							subPath = np.delete(subPath,0)
						path = np.insert(path,len(path),subPath)
					compTaken = 1
				else:
					path = np.insert(path,len(path),self.gotToWH(node, 1))
					compTaken = 0
				count += 1
		for node in path:
			print node.id


"""
#Dijkstra TESTS!!!!!
graph = Graph()
t0 = time.time()
graph.dijkstraDistancesNoQueue(0)
print "time dijkstra no queue", time.time()-t0

t1 = time.time()
graph.calculateDijkstraDistances(0,0)
print "time dijkstra queue", time.time()-t1"""



"""t0 = time.time()
graph = Graph()

graph.setRobotPosition(16,120)
graph.world.compWareHouse = [0,0,0,0,0,0]
graph.getOptimalPath([graph.world.availableProd[1], graph.world.availableProd[1]])"""

#path = graph.calculatePath()

"""agvPos = graph.getAGVPos()
print("agvPos:",agvPos)
graph.robotNode.x = agvPos[0]
graph.robotNode.y = agvPos[1]
graph.calculateDijkstraDistances(0, 0, alarm=True)

#graph.alarmActivated()

print("time:", time.time()-t0)

for node in path:
	#print("[",node.x, ",", node.y,"]")
	print(node.id)"""

