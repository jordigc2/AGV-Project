
import xml.etree.ElementTree as ET
import numpy as np
import csv

productDescrURL = "./Products_Composition/ProductsDescription.xml"
compDescrURL = "./Products_Composition/ComponentsDescription.xml"
prodListURL = "./Products_Composition/productionPlan.csv"

dicProd = {}
dicCompPos = {}

class Component():
	def __init__(self, _id, pos):
		self.compID = _id
		self.comPos = pos

class Product():
	def __init__(self, _id):
		self.prodID = _id
		self.compList = []

	def setComponents(self, compList):
		self.compList = compList

class World():
	def __init__(self):
		self.availableProd = []
		self.demandedProd = np.array([])

	def productsSetUp(self):

		prodDesc = ET.parse(productDescrURL)
		compDesc = ET.parse(compDescrURL)

		rootProd = prodDesc.getroot()
		rootComp = compDesc.getroot()

		for prod in rootProd:
			comArr = np.array([])
			prodID = prod.attrib["name"]
			product = Product(prodID)
			for comp in prod:
				compID = int(comp.text)-1
				xVal = int(rootComp[compID][0].text)
				yVal = int(rootComp[compID][1].text)
				component = Component(compID, np.array([xVal,yVal]))
				if comArr.size == 0:
					comArr = np.array([component])
				else:
					comArr = np.insert(comArr, len(comArr), component, 0)
			product.setComponents(comArr)
			self.availableProd.append(product)

	def readDemandedProductsCSV(prodListURL):
		pass



world = World()
world.productsSetUp()
world.readDemandedProductsCSV(prodListURL)


