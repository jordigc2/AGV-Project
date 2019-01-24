#!/usr/bin/env python 
from Tkinter import *
import numpy as np
import time
import random


listProd = np.array(["prod1","prod2","prod1","prod4"])
prodAssembling = "prod3"
prodAssembled = "prod1"
compRobot = [0,0]
compWH = [0,0,0,0,0,0]
x = 1.231
y = 2.321

currentNode = 0
goalNode = 1


class AGV5Inter(Frame):

    def __init__(self): 

        self.wndw = Tk()
        self.wndw.wm_title('AGV5 Tracker')
        self.wndw.config(background="#464B92")
        self.wndw.geometry("1080x650+0+0") 
        self.wndw.resizable(0, 0)
        self.value = 0
        self.elapTime = time.time()
        self.listProd = np.array(["prod1","prod2","prod1","prod4"])
        self.prodAssembling = "prod3"
        self.prodAssembled = "prod1"
        self.compRobot = [0,0]
        self.compWH = [0,0,0,0,0,0]
        self.x = 1.231
        self.y = 2.321
        self.currentNode = 0
        self.goalNode = 1

        self.build()
        self.insertMapImg('mapImg/map_original.gif')
        self.setValueTitles()
        self.loop()


    def setValuesStatus(listProd, prodAss, prodFin, comRob, comWH, x, y, currNode, goalNode):
        self.listProd = listProd
        self.prodAssembling = prodAss
        self.prodAssembled = prodFin
        self.compRobot = comRob
        self.compWH = comWH
        self.x = x
        self.y = y

        self.currentNode = currNode
        self.goalNode = goalNode

    def build(self):

        self.titleFrame = Frame(self.wndw,width = 200, height =50)#, background="#464B92")
        self.titleFrame.grid(row=0, column=0, padx=20, pady=10)

        titleTxt = Label(self.titleFrame, text="AVG5 Tracker", font=("Fixedsys", 20, "bold", "underline"))#, background="#F4D6BA")
        titleTxt.grid(row=0, column=0, padx=10, pady=2)

        self.topLeftFrame = Frame(self.wndw, width=200, height = 100)
        self.topLeftFrame.grid(row=1, column=0, padx=5, pady=2)


        self.bottomLeftFrame = Frame(self.wndw, width=200, height = 300)
        self.bottomLeftFrame.grid(row=2, column=0, padx=5, pady=5)

        self.topRightFrame = Frame(self.wndw, width=300, height = 300)
        self.topRightFrame.grid(row=1, column=1, padx=5, pady=5)


        self.bottomRightFrame = Frame(self.wndw, width=300, height = 400)
        self.bottomRightFrame.grid(row=2, column=1, padx=5, pady=5)


    def insertRobotImg(self, url):

        self.robotImg = PhotoImage(file = url)
        self.robotImg = self.robotImg.subsample(3)
        self.imgRobot = Label(self.topRightFrame, image=self.robotImg)
        self.imgRobot.image = self.robotImg
        self.imgRobot.grid(row=0, column=0)

    def selectMap(self):

        if self.currentNode == 0 or self.goalNode == 0:
            if self.currentNode != 0:
                url = 'mapImg/mapW_'+str(self.currentNode)+".gif"
            else:
                url = 'mapImg/mapW_'+str(self.goalNode)+".gif"
        elif self.currentNode <= 3 or self.goalNode <= 3:
            if self.currentNode < self.goalNode:
                url = 'mapImg/map'+str(self.currentNode-1)+"_p"+str(self.goalNode-3)+".gif"
            else:
                url = 'mapImg/map'+str(self.goalNode-1)+"_p"+str(self.currentNode-3)+".gif"
        elif self.currentNode <= 9 and self.goalNode <= 9 and (self.currentNode > 3 or self.goalNode > 3):
            if self.currentNode < self.goalNode:
                url = 'mapImg/map_p'+str(self.currentNode-3)+"_p"+str(self.goalNode-3)+".gif"
            else:
                url = 'mapImg/map_p'+str(self.goalNode-3)+"_p"+str(self.currentNode-3)+".gif"
        else:
            url = 'mapImg/map_original.gif'

        self.insertMapImg(url)

    def insertMapImg(self, url):

        self.mapImg = PhotoImage(file = url)
        self.mapImg = self.mapImg.subsample(2)
        self.imgMap = Label(self.bottomRightFrame, image=self.mapImg)
        self.imgMap.image = self.mapImg
        self.imgMap.grid(row=0, column=0)

    def setValueTitles(self):

        self.txtTitle0 = Label(self.topLeftFrame, text="Next Products:", font=("Fixedsys", 12, "bold"))
        self.txtTitle0.grid(row=0, column=0)

        self.txtTitle1 = Label(self.topLeftFrame, text="Product Assembling:", font=("Fixedsys", 12, "bold"))
        self.txtTitle1.grid(row=1, column=0)

        self.txtTitle2 = Label(self.topLeftFrame, text="Product Assembled:", font=("Fixedsys", 12, "bold"))
        self.txtTitle2.grid(row=2, column=0)

        self.txtTitle3 = Label(self.topLeftFrame, text="Components Robot:", font=("Fixedsys", 12, "bold"))
        self.txtTitle3.grid(row=3, column=0)

        self.txtTitle4 = Label(self.topLeftFrame, text="Components Warehouse:", font=("Fixedsys", 12, "bold"))
        self.txtTitle4.grid(row=4, column=0)

        self.txtTitle5 = Label(self.topLeftFrame, text="X position:", font=("Fixedsys", 12, "bold"))
        self.txtTitle5.grid(row=5, column=0)

        self.txtTitle6 = Label(self.topLeftFrame, text="Y position:", font=("Fixedsys", 12, "bold"))
        self.txtTitle6.grid(row=6, column=0)

        self.txtTitle7 = Label(self.topLeftFrame, text="TIME:", font=("Fixedsys", 12, "bold", "underline"))
        self.txtTitle7.grid(row=7, column=0)

        self.txtTitle8 = Label(self.bottomLeftFrame, text="Quality Check Decission", font=("Fixedsys", 16, "bold", "underline"))
        self.txtTitle8.grid(row=0, column=0)


        self.getSystemStatus()
        self.qcButton()


    def getSystemStatus(self):

        listProdTxt = ""
        for i in self.listProd:
            listProdTxt = listProdTxt + " " + i + " "

        robotCmp = ""
        for i in self.compRobot:
            robotCmp = robotCmp + " " + str(i) + " "


        whCmp = ""
        for i in self.compWH:
            whCmp = whCmp + " " + str(i) + " "

        t = int(time.time()-self.elapTime)

        self.txtValue0 = Label(self.topLeftFrame, text=listProdTxt, font=("Fixedsys", 12))
        self.txtValue0.grid(row=0, column=1)

        self.txtValue1 = Label(self.topLeftFrame, text=prodAssembled, font=("Fixedsys", 12))
        self.txtValue1.grid(row=1, column=1)

        self.txtValue2 = Label(self.topLeftFrame, text=prodAssembled, font=("Fixedsys", 12))
        self.txtValue2.grid(row=2, column=1)

        self.txtValue3 = Label(self.topLeftFrame, text=robotCmp, font=("Fixedsys", 12))
        self.txtValue3.grid(row=3, column=1)

        self.txtValue4 = Label(self.topLeftFrame, text=whCmp, font=("Fixedsys", 12))
        self.txtValue4.grid(row=4, column=1)

        self.txtValue5 = Label(self.topLeftFrame, text=str(self.x), font=("Fixedsys", 12))
        self.txtValue5.grid(row=5, column=1)

        self.txtValue6 = Label(self.topLeftFrame, text=str(self.y), font=("Fixedsys", 12))
        self.txtValue6.grid(row=6, column=1)

        self.txtValue7 = Label(self.topLeftFrame, text=str(t), font=("Fixedsys", 12))
        self.txtValue7.grid(row=7, column=1)

    def goodQC(self):

        self.button1.destroy()
        self.button2.destroy()

        self.txtQC = Label(self.bottomLeftFrame, text="No more products for QC", font=("Fixedsys", 12))
        self.txtQC.grid(row=1, column=0)


    def badQC(self):

        self.button1.destroy()
        self.button2.destroy()

        self.txtQC = Label(self.bottomLeftFrame, text="No more products for QC", font=("Fixedsys", 12))
        self.txtQC.grid(row=1, column=0)

    def qcButton(self):

        self.button1 = Button(self.bottomLeftFrame, text = "Good", command = self.goodQC)
        self.button1.grid(row=1, column=0, pady = 5)

        self.button2 = Button(self.bottomLeftFrame, text = "Bad", command = self.goodQC)
        self.button2.grid(row=2, column=0, pady = 5)

    def destroyWidgets(self):

        self.imgRobot.destroy()
        self.imgMap.destroy()
        self.txtValue0.destroy()
        self.txtValue1.destroy()
        self.txtValue2.destroy()
        self.txtValue3.destroy()
        self.txtValue4.destroy()
        self.txtValue5.destroy()
        self.txtValue6.destroy()
        self.txtValue7.destroy()


    def loop(self):

        self.currentNode = random.randint(1,3)
        self.goalNode = random.randint(4,9)
        if self.value != 0:
            pass
            #self.destroyWidgets()
        if self.value%2 == 0:
            self.insertRobotImg('robotImg/robot2.gif')
            self.listProd = np.array(["prod1","prod2","prod1","prod4"])
        else:
            self.insertRobotImg('robotImg/robot.gif')
            self.listProd = np.array(["prod4","prod2","prod2","prod3"])
        self.selectMap()
        self.getSystemStatus()
        self.value += 1
        self.wndw.after(900,self.loop)


    def closeWndw(self):

        self.wndw.destroy()
        exit()
    def run(self):
        
        self.wndw.protocol("WM_DELETE_WINDOW",self.closeWndw)
        self.wndw.mainloop()


if __name__ == '__main__':

    window = AGV5Inter()
    window.run()

    a = True

    while True:
        if a:
            window.insertRobotImg('robotImg/robot2.gif')

        else:
            window.insertRobotImg('robotImg/robot.gif')

        a = not a

        sleep(1)

