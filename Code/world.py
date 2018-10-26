
import xml.etree.ElementTree as ET
import numpy as np
import csv

productDescrURL = "./Products_Composition/ProductsDescription.xml"
compDescrURL = "./Products_Composition/ComponentsDescription.xml"
prodListURL = "./Products_Composition/productionPlan.csv"
objectsDescrURL = "./Products_Composition/mapPositions.xml"

dicProd = {}
dicCompPos = {}

class Component():
	def __init__(self, _id, pos):
		self.compID = _id
		self.comPos = pos
		self.collected = False
		self.prodID = -1 #usful to make sure that all the components of the same product are together

class Product():
	def __init__(self, _id):
		self.prodID = _id
		self.compList = []

	def setComponents(self, compList):
		self.compList = compList

class World():
	def __init__(self):
		print("Creating the World")
		self.availableProd = []
		self.availableComp = []
		self.demandedProd = np.array([], dtype=int)
		self.componentsList = np.array([], dtype='object')
		self.objectsPos = []
		self.numObjects = 0

		self.componentsSetUp()
		self.productsSetUp()
		self.readDemandedProductsCSV(prodListURL)
		self.setComponentsList()
		self.setWorldObjects()


	def componentsSetUp(self):	
		#create the components objects acording to the XML document information
		compDesc = ET.parse(compDescrURL)
		rootComp = compDesc.getroot()
		count = 0
		for comp in rootComp:
			xVal = int(comp[0].text)
			yVal = int(comp[1].text)
			component = Component(count+1, np.array([xVal,yVal]))
			self.availableComp.append(component)
			count += 1

	def productsSetUp(self):
		#create the products objects acording to the XML document information

		prodDesc = ET.parse(productDescrURL)
		compDesc = ET.parse(compDescrURL)

		rootProd = prodDesc.getroot()
		rootComp = compDesc.getroot()

		count = 1
		for prod in rootProd:
			comArr = np.array([])
			prodID = prod.attrib["name"]
			product = Product(prodID)
			for comp in prod:
				compID = int(comp.text)-1
				if comArr.size == 0:
					comp = self.availableComp[compID]
					comp.prodID = count
					comArr = np.array([comp])
				else:
					comp = self.availableComp[compID]
					comp.prodID = count
					comArr = np.insert(comArr, len(comArr), comp, 0)
			product.setComponents(comArr)
			self.availableProd.append(product)
			count += 1

	def readDemandedProductsCSV(self, prodListURL):
		#create the list of all the Products on the CSV

		with open(prodListURL) as csvFile:
			reader = csv.reader(csvFile)
			next(reader)
			for prodId in reader:
				self.demandedProd = np.insert(self.demandedProd, len(self.demandedProd), prodId[0])
			#self.demandedProd = self.demandedProd.astype(int)

	def setComponentsList(self):
		#create the list of all the Components needed to create the Products
		for prodID in self.demandedProd:
			prodCompList = self.availableProd[prodID-1].compList
			self.componentsList = np.insert(self.componentsList, len(self.componentsList), prodCompList)


	def setWorldObjects(self):
		#Create the list of positions of all the objects that are in the map such as warehouse or walls gaps
		objDesc = ET.parse(objectsDescrURL)
		objRoot = objDesc.getroot()
		self.numObjects = len(objRoot)
		for obj in objRoot:
			pos = [int(obj[0].text), int(obj[1].text)]
			self.objectsPos.append(pos)