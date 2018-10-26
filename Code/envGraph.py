import sys
import world as wd



class Node:
	def __init__(self,x,y, nodeId):
		self.x = x
		self.y = y
		self.id = nodeId
		self.neigh = []
		self.prevNeigh = 0
		self.statDistance = sys.maxsize #set a huge number to the distances
		self.dynDistance = sys.maxsize #set a huge number to the distances
		self.visited = False


class Graph:
	def __init__(self):
		self.listOfNodes = []

		self.world = wd.World()

		count = 0
		#creation of the nodes of the graph
		for obj in self.world.objectsPos:
			node = Node(obj[0],obj[1], count)
			self.listOfNodes.append(node)
			count += 1

		for comp in self.world.availableComp:
			node = Node(comp.comPos[0], comp.comPos[1], comp.compID+count)
			self.listOfNodes.append(node)
			count += 1

		self.setNeighbours()

	def setNeighbours(self):

		count = 0
		for node in self.listOfNodes:
			if count == 0:
				#set warehouse neighbours
				for aux in self.listOfNodes[1:self.world.numObjects]:
					node.neigh.append(aux)
			elif count >= 1 and count <= self.world.numObjects:
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
		print([self.listOfNodes[0].neigh[0].x,self.listOfNodes[0].neigh[0].y])



graph = Graph()

