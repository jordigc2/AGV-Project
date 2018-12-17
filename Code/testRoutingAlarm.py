import envGraph as eg
import numpy as numpy
import copy

compWareHouse = [0,0,0,0,0,0]
compRobot = [3,3]
avComp = [1,1]
posRobot = [120,180]

graph = eg.Graph()


print("Initializing the world")
graph.setRobotPosition(posRobot[0],posRobot[1])
path = graph.calculatePath(alarm=True)

print("Current path\n")
for node in path[0:15]:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID )
	else:
		print("nID:",node.id)
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
		print("nID:",node.id)


