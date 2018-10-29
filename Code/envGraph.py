import sys
import world as wd
import bisect
import math
import time



class Node:
	def __init__(self,x,y, nodeId):
		self.x = x
		self.y = y
		self.id = nodeId
		self.neigh = []
		self.prevNeigh = 0
		self.dijkstraDistance = sys.maxsize #set a huge number to the distances
		self.visited = False

	def __lt__(self, other):
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
			node = Node(comp.comPos[0], comp.comPos[1], comp.compID+count-1)
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

graph = Graph()

graph.calculateDijkstraDistances(0,0)