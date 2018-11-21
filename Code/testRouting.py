import envGraph as eg
import time
import numpy as np

TIME_OBJECT = 2
TIME_COMPONENT = 5
TIME_PRODUCT = 10
TIME_QC = 5
t0 = time.time()
graph = eg.Graph()


print("Initializing the world")
graph.setRobotPosition(50,100)
path = graph.calculatePath()

print("time initialation:", time.time()-t0)
elemRobot = 0

"""for prod in graph.world.availableProd:
	print("prodID", prod.prodID)
	for comp in prod.compList:
		print("compID:", comp.compID," prodID:", comp.prodID," memory:", comp)"""
"""for node in path:
	#print("[",node.x,", ",node.y,"]")
	if node.component != -1:
		print("nodeID:",node.id, "CompProdID: ",node.component.prodID)
	else:
		print("nodeID:",node.id)"""

print("")
print("--------")
graph.world.productsList[0].inProgress = True
productDelivered = False
timeAssembly = 0
prodAssembled = False
qcResult = True
while len(path) > 0:
	if timeAssembly!= 0:
		timePassed = time.time()-timeAssembly
		print("Time: ", timePassed)
		if timePassed >= TIME_PRODUCT:
			print("Assembly product", graph.world.prevProdDone.prodID)
			for comp in graph.world.prevProdDone.compList:
				graph.world.compWareHouse[comp.compID-1] -= 1
			print("Components in WareHouse:", graph.world.compWareHouse)
			timeAssembly = time.time()
			#timePassed = 0
			prodAssembled = True
		if timePassed >= TIME_QC and prodAssembled:
			qc = input("press 1 for OK or 0 for defect product and 'ENTER':\n")
			timeAssembly = 0
			if qc == 0:
				qcResult = False
			else:
				qcResult = True
			print("Quality Check result",qcResult)
	if not qcResult:
		print("productDefected")
		graph.robotNode.x = actNode.x
		graph.robotNode.y = actNode.y
		path = graph.alarmActivated()
		#print(path)
		qcResult = True
	actNode = path[0]
	path = np.delete(path,0)
	print("Actual Node:", actNode.id)
	if actNode.id<4:
		if actNode.id == 0:
			if productDelivered:
				timeAssembly = time.time()
				prodAssembled = False
				productDelivered = False
			if sum(graph.world.compRobot)>0:
				count = 0
				for compID in graph.world.compRobot:
					graph.world.compWareHouse[compID-1]+=1
					print("Leaving Component",compID," to WareHouse")
					graph.world.compRobot[count] = 0
					count += 1
				elemRobot = 0
				time.sleep(TIME_OBJECT)
			print("WareHouse has this components:", graph.world.compWareHouse)
		time.sleep(TIME_OBJECT)
	elif actNode.id < 10:
		product = graph.world.productsList[0]
		product.numCompTaken += 1
		print("picking component", actNode.component.compID)
		graph.world.compRobot[elemRobot] = actNode.component.compID
		print("num Components taken for this product", product.numCompTaken)
		elemRobot += 1
		print("Robot has this components:", graph.world.compRobot)
		if product.numCompTaken == len(product.compList):
			product.numCompTaken = 0
			product.inProgress = False
			print("product:",product.prodID,"DONE")
			graph.world.prevProdDone = product
			print("prevProdDone:", graph.world.prevProdDone.prodID)
			graph.world.productsList = np.delete(graph.world.productsList,0)
			print("Setting product:",graph.world.productsList[0].prodID,"in progress=True")
			print("Comp taken:",graph.world.productsList[0].numCompTaken, graph.world.productsList[0])
			graph.world.productsList[0].inProgress = True
			productDelivered = True

		
		time.sleep(TIME_COMPONENT)
	print("--------")

print("time:", time.time()-t0)



