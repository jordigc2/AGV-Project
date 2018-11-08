import sys
import world as wd
import math
import time
import bisect
import numpy as np
from operator import attrgetter



class Node:
	def __init__(self,x, y, _id, component = -1):
		"""
			Param: x, y position of the element, _id of the element, from 0-3 objects in the world and from 4-9 components and component is the Component object
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
		 Description: Used to have a list of Nodes sorted by the dijkstraDistance attribute using the library bisect
		"""
		return(self.dijkstraDistance < other.dijkstraDistance)


class Graph:
	def __init__(self):
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
		print([self.listOfNodes[0].neigh[0].x,self.listOfNodes[0].neigh[0].y])

	def calculateDijkstraDistances(self, startingNodeID, initDistance):

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
		compNodes = []
		sortedCompList = [0]*len(product.compList)
		for comp in product.compList:
			node = self.listOfNodes[comp.compID+self.world.numObjects-1]
			bisect.insort_left(compNodes, node)

		return compNodes

	def gotToWH(self, node, direction):
		subPath = np.array([node])
		while node.prevNeigh != 0:#insert to the subPath until reaching to the WH
			node = node.prevNeigh
			if direction == 0:#from WH to node
				subPath = np.insert(subPath,0, node)
			else:#from node to WH
				subPath = np.insert(subPath,len(subPath), node)
		return subPath

	def getClosestComp(self, nodeList, actNode):
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

	#Calculate all the steps to do for all the components needed.
	def calculatePath(self, processItems = 5):
		path = np.array([], dtype="object") #numpy array of nodes
		optimalDistance = sys.maxsize
		currentDistance = 0
		compTaken = 0
		actNode = self.listOfNodes[1]
		for product in self.world.productsList:
			product.uncollectComp()

			if(len(path)>0):
				actNode = path[len(path)-1]
			prodPath,compTaken = self.generatePath(product, compTaken, actNode)

			path = np.insert(path,len(path), prodPath)
		if(compTaken == 1):
			lastComp = path[len(path)-1]
			path = np.delete(path,len(path)-1)
			path = np.insert(path,len(path), self.gotToWH(lastComp,1)) 
		return path




t0 = time.time()
graph = Graph()

graph.calculateDijkstraDistances(0,0)
path = graph.calculatePath()

print("time:", time.time()-t0)

for node in path:
	print(node)

