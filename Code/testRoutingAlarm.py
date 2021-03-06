import envGraph as eg
import numpy as np
import copy
import math as m
import matplotlib.pyplot as plt

compWareHouse = [0,0,0,0,0,0]
compRobot = [5,3]
avComp = [1,0]
posRobot = [120,120]

graph = eg.Graph()
path = np.array([], dtype="object")

print("Initializing the world")
graph.setRobotPosition(posRobot[0],posRobot[1])
path = graph.calculatePath(alarm=True)

"""
print ("\n_______________________________________")
graph.world.prevProdDone = copy.copy(graph.world.productsList[9])#product4
graph.world.productsList[0].inProgress = True
#print(graph.world.productsList[0].compIDList)
graph.robotNode.x = posRobot[0]
graph.robotNode.y = posRobot[1]
graph.world.compRobot = compRobot
graph.world.compAvRobot = avComp
graph.world.compWareHouse = compWareHouse

path = graph.alarmActivated(mode = 0)
print("_______________________________________\n")
print("Path after a defected product\n")
for node in path[0:18]:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID, "cReturn", node.component.returnComp )
	else:
		print("nID:",node.id)"""


