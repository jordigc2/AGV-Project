import envGraph as eg
import numpy as np
import copy
import math as m
import matplotlib.pyplot as plt

compWareHouse = [0,0,0,2,0,0]
compRobot = [5,3]
avComp = [1,0]
posRobot = [150,180]

graph = eg.Graph()
path = np.array([], dtype="object")
graph.world.compWareHouse = compWareHouse


print("Initializing the world")
graph.setRobotPosition(posRobot[0],posRobot[1])
path1 = graph.calculatePath(alarm=True)
count = 1
for product in graph.world.productsList:
	if count < len(graph.world.productsList):
		path = graph.getOptimalPath(path, [product,graph.world.productsList[count]])
	else:
		path = graph.getOptimalPath(path, [product])
	graph.setRobotPosition(path[len(path)-1].x, path[len(path)-1].y)
	count += 1

print("\nGreedy path")
print("Components WH:", compWareHouse)
for node in path1[0:10]:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID )
	else:
		print("nID:",node.id)

print ("--------------")
print("\nOptimal path")
print("Components WH:", compWareHouse)
for node in path[0:10]:
	if node.component != -1:
		print("nID:",node.id,"cID:", node.component.compID,"pID:",node.component.prodID )
	else:
		print("nID:",node.id)

preNode = 0
distance = 0
listDist = []
listDist2 = []

for node in path:
	if preNode != 0:
		distance += m.sqrt((preNode.x-node.x)**2+(preNode.y-node.y)**2)
	preNode = node
	listDist.append(distance)
print("totalDistance optimal:", distance)
distance = 0
preNode = 0
for node in path1:
	if preNode != 0:
		distance += m.sqrt((preNode.x-node.x)**2+(preNode.y-node.y)**2)
	preNode = node
	listDist2.append(distance)
print("totalDistance greedy:", distance)

plt.title("Greedy algorithm vs optimal path algorithm", fontsize=16)
plt.plot(listDist, label="Optimal path")
plt.plot(listDist2, label="Greedy path")
plt.ylabel("Distance", fontsize=13)
plt.xlabel("Steps(t)", fontsize=13)
plt.legend(prop={'size': 10})
plt.show()
