#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Pose2D
from AGV5.msg import target
from AGV5.msg import robot, goalPos
from tf import transformations as t
from math import degrees, atan, pi



def callback(data):

	global r_x
	global r_y
	global r_theta
	

	r_x = data.x_pos
	r_y = data.y_pos
	r_theta = data.eul_pos


def getGoal(data):
	global goalX
	global goalY
	
	goalX = data.x_pos
	goalY = data.y_pos
	print("GOAL X: %f, Y: %f" %(goalX, goalY) )  

def calc_angle(cx, cy, ctheta, gx, gy, gtheta):

	pub = rospy.Publisher("/some/topic", target, queue_size=1000)
	delta_x = gx - cx
	delta_y = gy - cy

	kp = 0.8

	msg = target()
	print("my goal is : ", gx, gy)

	if not delta_x == 0:

	#	print("Detla X WAS NOT 0") 
		if 0 > delta_x and 0 < delta_y:
			vtheta = pi + atan(delta_y/delta_x)
		elif 0 < delta_x and 0 > delta_y:
			vtheta = atan(delta_y/delta_x)
		elif 0 > delta_x and 0 > delta_y:
			vtheta = atan(delta_y/delta_x) - pi
		else: 
			vtheta = atan(delta_y/delta_x)

		if 0 < ctheta and ctheta > vtheta + pi:
			angle = vtheta - ctheta + 2*pi
		else:
			angle = vtheta - ctheta 

		msg.linvel = 0.2
		msg.angvel = angle * kp

		vr = (2*msg.linvel + msg.angvel*0.125) / (2*0.02) 
		vl = (2*msg.linvel - msg.angvel*0.125) / (2*0.02)
		#print("rVel = %f" % vr)
		#print("lVel = %f" % vl)

		pub.publish(msg)


r_x = 0
r_y = 0
r_theta = 0

goalX = 0
goalY = 0


rospy.init_node("robotController")
rospy.Subscriber("robotPos", robot, callback)
rospy.Subscriber("goalPos", goalPos, getGoal)
r = rospy.Rate(40)

while not rospy.is_shutdown():

	calc_angle(r_x, r_y, r_theta, goalX, goalY, 0)
	r.sleep()
