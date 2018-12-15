#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from AGV5.msg import robot
import rospy

x_array = []
y_array = []

def plot_RobotPos(data):
	x_array.append(round(data.x_pos,2))
	y_array.append(round(data.y_pos,2))

def update(i):
	ax.clear()
	ax.plot(x_array, y_array)

	# Format plot
	plt.title('Real time robot position')
	plt.ylabel('y position')
	plt.xlabel('x position')

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
rospy.init_node('plotter', anonymous=True)
rospy.Subscriber('robotPos', robot, plot_RobotPos)

if __name__ == '__main__':

	ani = animation.FuncAnimation(fig, update, interval=1)
	plt.show()
	rospy.spin()