import envGraph as eg
import numpy as numpy
import copy

compWareHouse = [0,0,0,0,0,0]
compRobot = [0,0]
avComp = [0,0]
posRobot = [16,127]

graph = eg.Graph()


print("Initializing the world")
graph.setRobotPosition(16,120)
path = graph.calculatePath()

"""for node in path:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID )
	else:
		print("nID:",node.id)"""

graph.world.prevProdDone = copy.copy(graph.world.productsList[9])#product4
graph.world.productsList[0].inProgress = True
print(graph.world.productsList[0].compIDList)
graph.robotNode.x = posRobot[0]
graph.robotNode.y = posRobot[1]
graph.world.compRobot = compRobot
graph.world.compAvRobot = avComp
graph.world.compWareHouse = compWareHouse

path = graph.alarmActivated()

for node in path[0:20]:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID )
	else:
		print("nID:",node.id)


