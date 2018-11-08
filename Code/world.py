
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

	def uncollectComp(self):
		for comp in self.compList:
			comp.collected = False

class World():
	def __init__(self):
		"""
			CONSTRUCTOR
			Param: None
			Return: None
			Description: Read all the files to create the components and products and set the world objects and init the WH and robot components as empty.
		"""
		print("Creating the World")
		self.availableProd = []
		self.availableComp = []
		self.demandedProd = np.array([], dtype=int)
		self.componentsList = np.array([], dtype='object')
		self.productsList = np.array([], dtype='object')
		self.objectsPos = []
		self.numObjects = 0

		self.compWareHouse = [0]*6 #In each position it can bee up to 3 components
		self.compRobot = [0]*2 #Each position is the ID of a component node in the graph
		self.prevProdDone = -1 #ID of a product to know wich products are the ones needed again

		self.componentsSetUp()
		self.productsSetUp()
		self.readDemandedProductsCSV(prodListURL)
		self.setComponentsList()
		self.setWorldObjects()


	def componentsSetUp(self):	
		"""
			Param: None
			Return: None
			Description: Creates all the available Components and store the objects in the list	availableComp from the class world. Create the components objects acording to the XML document information
		"""

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
		"""
			Param: None
			Return: None
			Description: Create the available Products objects according to the XML document information and puts the components into the list of compList of the class Product
		"""

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
				comp = self.availableComp[compID]
				comp.prodID = count
				if comArr.size == 0:
					comArr = np.array([comp])
				else:
					comArr = np.insert(comArr, len(comArr), comp, 0)
			product.setComponents(comArr)
			self.availableProd.append(product)
			count += 1

	def readDemandedProductsCSV(self, prodListURL):
		"""
			Param: prodListURL, the URL of the CSV file
			Return: None
			Description: Add the demanded products from the csv to the demandedProd list of the class World.
		"""

		with open(prodListURL) as csvFile:
			reader = csv.reader(csvFile)
			next(reader)
			for prodId in reader:
				self.demandedProd = np.insert(self.demandedProd, len(self.demandedProd), prodId[0])
			#self.demandedProd = self.demandedProd.astype(int)

	def setComponentsList(self):
		"""
			Param: None
			Return: None
			Description: According to the list demandedProd it adds the Components to the componentsList of the class World
		"""
		#create the list of all the Components needed to create the Products
		for prodID in self.demandedProd:
			self.productsList = np.insert(self.productsList, len(self.productsList), self.availableProd[prodID-1])
			prodCompList = self.availableProd[prodID-1].compList
			self.componentsList = np.insert(self.componentsList, len(self.componentsList), prodCompList)


	def setWorldObjects(self):
		"""
			Param: None
			Return: None
			Description: Puts the position of the different object in the map according to the XML file. The first one is the WareHouse and the 3 last the middle points of the walls.
		"""
		#Create the list of positions of all the objects that are in the map such as warehouse or walls gaps
		objDesc = ET.parse(objectsDescrURL)
		objRoot = objDesc.getroot()
		self.numObjects = len(objRoot)
		for obj in objRoot:
			pos = [int(obj[0].text), int(obj[1].text)]
			self.objectsPos.append(pos)