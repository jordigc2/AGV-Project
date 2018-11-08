import sys
import world as wd
import math
import time
import bisect
import numpy as np
import random
from operator import attrgetter



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
		self.dijkstraDistance = sys.maxsize #set a huge number to the distances
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
		return(self.dijkstraDistance < other.dijkstraDistance)


class Graph:
	def __init__(self):
		"""
		Param: None
		Return: None
		Description: Initialize the mapping of the real map into a graph. First the world objects
		are inserted to the graph and then the nodes corresponding to the 6 components.
		"""

		self.listOfNodes = [] #0  WH, 1-3 wallnodes and 4-9 components nodes

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

	def calculateDijkstraDistances(self, startingNodeID, initDistance):
		"""
			Param: startingNodeID, this node will have dijkstraDistance = 0; initDistance is to know
			how far is the robot from the starting node.
			Return: None
			Description: Run over all the nodes of the graph using a priority queue in order to go 
			faster and calculate the shortest distance from the starting node.
		"""

		startNode = self.listOfNodes[startingNodeID]
		startNode.dijkstraDistance = initDistance
		priorityQueue = [startNode]

		while len(priorityQueue) > 0:
			actualNode = priorityQueue.pop(0)
			if not actualNode.visited:

				for node in actualNode.neigh:
					dist = actualNode.dijkstraDistance + math.sqrt((node.x-actualNode.x)**2+(node.y-actualNode.y)**2)
					if dist < node.dijkstraDistance:
						node.dijkstraDistance = dist
						node.prevNeigh = actualNode
					bisect.insort_left(priorityQueue, node)

				actualNode.visited = True

	def sortCompDist(self, product):
		"""
			Param: product, to be able to get the list of components needed.
			Return: compNodes, is a list of sorted nodes corresponing to a component.
			Description: Using the bisect libary, a list of nodes is created but the nodes are
			placed sorted inside the list.
		"""

		compNodes = []
		sortedCompList = [0]*len(product.compList)
		for comp in product.compList:
			node = self.listOfNodes[comp.compID+self.world.numObjects-1]
			bisect.insort_left(compNodes, node)

		return compNodes

	def gotToWH(self, node, direction):
		"""
			Param: node, is the actual node that the robot is and direction, to know if the path 
			goes from the WH to the node or the other way around.
			Return: subPath
			Description: Generates a subPath to be able to go or return from a node from the WH.
		"""
		subPath = np.array([node])
		while node.prevNeigh != 0:#insert to the subPath until reaching to the WH
			node = node.prevNeigh
			if direction == 0:#from WH to node
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


	def generatePath(self, product, compTaken, actNode):
		"""
			Param: product, used to get the components needed, compTaken, how many components is 
			planned to have the robot at each moment, actNode, to know the path to the next object.
			Return: subPath and compTaken
			Description: It creates the path needed to get all the components of a product. It is 
			checked how many comp is planned to have the robot at each moment. Because, after 
			getting all the components of a product, there might be an empty gap, which will be 
			used with one of the component of the following component.
		"""

		nodesComp = self.sortCompDist(product)
		"""print("Components Sorted")
		for node in nodesComp:
			print(node.id)"""
		path = np.array([], dtype="object")
		pathCreated = False
		FROM_HOME = 0
		FROM_NODE = 1
		while not pathCreated:
			if compTaken == 1:
				closestNode = self.getClosestComp(nodesComp, actNode)
				if closestNode != 0:
					path = np.insert(path,len(path),self.gotToWH(closestNode, FROM_NODE))
					closestNode.component.collected = True
					compTaken = 0
			else:
				for node in nodesComp:
					if not node.component.collected:
						#print("FirstComonent To take:", node.id)
						node.component.collected = True
						subPath = self.gotToWH(node, FROM_HOME)
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
						#if(closestNode != 0):
						#	print("Closest:",closestNode.id)
						if closestNode != 0:
							path = np.insert(path,len(path),self.gotToWH(closestNode, FROM_NODE))
							closestNode.component.collected = True
							compTaken = 0
				pathCreated = True

		return path,compTaken

	def calculatePath(self, alarm = False, processItems = 5):
		"""
			Param: processItems, used when the following products are taken into consideration, to
			know if there is any alarm or not.
			Return: path, path at time zero.
			Description: It goes trough all the products and it generates the path of how to reach 
			to evry single component in order to get the products assembled.
		"""

		path = np.array([], dtype="object") #numpy array of nodes
		optimalDistance = sys.maxsize
		currentDistance = 0
		compTaken = 0
		actNode = self.listOfNodes[1]
		for product in self.world.productsList:
			if not alarm:
				product.uncollectComp()
			alarm = False

			if(len(path)>0):
				actNode = path[len(path)-1]
			prodPath,compTaken = self.generatePath(product, compTaken, actNode)

			path = np.insert(path,len(path), prodPath)
		if(compTaken == 1):
			lastComp = path[len(path)-1]
			path = np.delete(path,len(path)-1)
			path = np.insert(path,len(path), self.gotToWH(lastComp,1)) 
		return path

	def getAGVPos(self):
		"""
			Param: None
			Return: Position X,Y
			Description: Now it returns random values inside the map coordenates, but in a future,
			will return the topic position published by VICON.
		"""
		return [random.randint(10,235),random.randint(62,207)]

	def alarmActivated(self):
		"""
			Param: None
			Return: updatedPath
			Description: This function runs when a product is defected and updates the path in order
			to get the components to redo the product. This function will read from the ROS topics 
			of the components in the warehous, the components in the robot and the position of the
			robot.
		"""
		robotPos = self.getAGVPos()
		prodDefect = self.world.prevProdDone
		comList = prodDefect.compList
		prodDefect.uncollectComp()
		count = 0
		if sum(world.compRobot) == 0: 
			#robot has no components
			for comp in comList:
				if world.compWareHouse[comp.compID] > 0:
					comp.collected = True

			self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
			path = calculatePath(alarm=True)
		else:
			if robotPos[1] <= 164: #robot in sector1
				for comp in comList:
					if comp.compID in world.compRobot or world.compWareHouse[comp.compID] > 0:
						comp.collected = True
				self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
				path = calculatePath(alarm=True)
			else:
				compRobNeed = 0
				indxSame = -1
				for comp in comList:
					if comp.compID in world.compRobot:
						aux = world.compRobot.index(comp.compID)
						if aux != indxSame:
							comp.collected = True
							indxSame = aux
							compRobNeed += 1
					elif world.compWareHouse[comp.compID] > 0:
						comp.collected = True

				if compRobNeed == 2:
					self.world.productsList = np.insert(self.world.productsList, 0, prodDefect)
					path = calculatePath(alarm=True)
				else: #leave the components not needed and pick the ones needed
					pass



t0 = time.time()
graph = Graph()

graph.calculateDijkstraDistances(0,0)
path = graph.calculatePath()

graph.alarmActivated()

print("time:", time.time()-t0)

"""for node in path:
	print(node)"""

