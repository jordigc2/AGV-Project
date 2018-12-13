import envGraph as graph
import itertools

def getOptimalPath(graph, listProducts):
	actProduct = listProducts[0]
	listProducts = np.delete(listProducts,0)
	#prodPermut = 



def getPaths(graph, numProducts = 10):

	listProducts = graph.world.productsList[0:10]
	optPath = getOptimalPath(graph, listProducts)

