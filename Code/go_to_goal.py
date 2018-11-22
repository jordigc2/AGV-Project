#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped, Pose2D
from AGV5.msg import target				
from AGV5.msg import robot
#from tf import transformations as t
from math import degrees, atan, pi, sqrt


def callback(data):
	global r_x
	global r_y
	global r_theta
	
	r_x = data.x_pos
	r_y = data.y_pos
	r_theta = data.eul_pos


def getGoal(data):
	global new_goalX
	global new_goalY
	global new_mode 

	new_goalX = data.x_pos
	new_goalY = data.y_pos
	new_mode = data.eul_pos
	print("GOAL X: %f, Y: %f" %(new_goalX, new_goalY) )  
	print("MOD:" , new_mode)


def calc_angle(cx, cy, ctheta, gx, gy, mode):
	global goal_reached

	pub = rospy.Publisher("/some/topic", target, queue_size=1000)

	anglim = 2.0
	linlim = 0.3

	kp = 2
	kp_lin = 0.5

	delta_x = gx - cx
	delta_y = gy - cy

	msg = target()

#-----------------------------Calc Angle Error--------------------------------

	if not delta_x == 0:

		if 0 > delta_x and 0 < delta_y:
			vtheta = pi + atan(delta_y/delta_x)
		elif 0 < delta_x and 0 > delta_y:
			vtheta = atan(delta_y/delta_x)
		elif 0 > delta_x and 0 > delta_y:
			vtheta = atan(delta_y/delta_x) - pi
		else: 
			vtheta = atan(delta_y/delta_x)

		if ctheta < vtheta - pi :
			angle = vtheta - ctheta - 2*pi
		elif ctheta > vtheta + pi : 
			angle = vtheta - ctheta + 2*pi
		else:
			angle = vtheta - ctheta  

		print(vtheta, ctheta, degrees(angle))
#---------------------------Velocity assigment---------------------------------
		

		angvel = angle * kp

		if angvel > anglim:
			angvel = anglim
		elif angvel < -anglim:
			angvel = -anglim


		linvel = linlim - (linlim / anglim) * abs(angvel)

#--------------------------------Stopping--------------------------------------

		dist = sqrt( delta_x**2 + delta_y**2 )


		print("I was called", gx, gy, mode)
		#print("DISTANCE TO Goal :", dist)

		if  dist < break_length and mode != 0:
			linvel = dist * kp_lin

		else:
			linvel = max_linvel

		if dist < 0.080:
			if mode != 0: 
				linvel = 0
				angvel = 0
				
				msg.linvel = linvel
				msg.angvel = angvel

				pub.publish(msg)
				rospy.sleep(5)
			goal_reached = True


		msg.linvel = linvel
		msg.angvel = angvel

		pub.publish(msg)

		vr = (2*msg.linvel + msg.angvel*0.125) / (2*0.02) 
		vl = (2*msg.linvel - msg.angvel*0.125) / (2*0.02)
		#print("rVel = %f" % vr)
		#print("lVel = %f" % vl)
			
#------------------------------Main---------------------------------

goal_reached = True

r_x = 0
r_y = 0
r_theta = 0

goalX = 30
goalY = 30
mode = 0

new_goalX = 30
new_goalY = 30
new_mode = 0

max_linvel = 0.3
break_length = 0.26

rospy.init_node("robotController")
rospy.Subscriber("robotPos", robot, callback)
gsub = rospy.Subscriber("goalPos", robot, getGoal)
r = rospy.Rate(10)

rospy.sleep(2)


while not rospy.is_shutdown():
	#print ("goal reached, new x and new y : " ,goal_reached, new_goalX, new_goalY)
	if goal_reached == True and (new_goalX != goalX or new_goalY != goalY):
		print "NEW GOAL!!!"
		goalX = new_goalX
		goalY = new_goalY
		mode = new_mode
		goal_reached = False

	calc_angle(r_x, r_y, r_theta, goalX, goalY, mode)
	r.sleep()
