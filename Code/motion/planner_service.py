#!/usr/bin/env python
import rospy
from planner_pkg.srv import *
import math
import numpy as np
from planner_pkg.msg import robot as robot_msg
from tf import transformations as tf
from nav_msgs.msg import Odometry


class robot_instance:
	def __init__(self):
		self.pos = self.position(0,0)
		self.ori = self.orientation(0)
		self.vel = self.velocity()
		self.lim = self.limits(2.5, 3.33, 9.2, 5.71)  #Set these according to robot constraintes

	class position:
		def __init__(self, a, b):
			self.x = a
			self.y = b
	class orientation:
		def __init__(self,a):
			self.theta = a
	class velocity:
		def __init__(self):
			self.lin = 0
			self.ang = 0
	class limits:
		def __init__(self,a,b,c,d):
			self.v = a
			self.a = b
			self.w = c
			self.w_dot = d
	def move(self, cmd, timeInterval):
		#Angular velocity change with limits
		if abs(cmd.ang - self.vel.ang) < self.lim.w_dot * timeInterval:
			self.vel.ang = cmd.ang  
		else:
			self.vel.ang += math.copysign(self.lim.w_dot*timeInterval, cmd.ang) 
		#Linear velocity change with limits
		if abs(cmd.lin - self.vel.lin) < self.lim.a * timeInterval:
			self.vel.lin = cmd.lin  
		else:
			self.vel.lin += math.copysign(self.lim.a*timeInterval, cmd.lin - self.vel.lin)
		#Orientation change
		self.ori.theta += self.vel.ang * timeInterval
		self.ori.theta = wrap2Pi(self.ori.theta)
		#Position change
		self.pos.x += self.vel.lin * math.cos(self.ori.theta) * timeInterval
		self.pos.y += self.vel.lin * math.sin(self.ori.theta) * timeInterval
	def get_minStopTime(self):
		return abs(self.vel.lin) / self.lim.a if self.vel.lin != 0 else 1.0
	def get_minStopDistance(self):
		return abs(self.vel.lin * self.get_minStopTime() / 2)
	def get_maxStopVelocity(self):
		s = abs(self.get_minStopDistance())
		return s/ math.sqrt(2*s/self.lim.a) if s != 0 else self.lim.v
	def get_minStopTimeAngle(self):
		return abs(self.vel.ang) / self.lim.w_dot if self.vel.ang != 0 else 1.0
	def get_minStopAngle(self):
		return self.vel.ang * self.get_minStopTimeAngle() / 2
	def get_maximumAngleVel(self, target_angle):
		return target_angle / math.sqrt(2*abs(target_angle)/self.lim.w_dot)

class point_instance:
	def __init__(self, x_p, y_p):
		self.x = x_p
		self.y = y_p

class command():
	def __init__(self,a,b):
		self.lin = a
		self.ang = b

def wrap2Pi(angle):
	angle = ( angle + np.pi) % (2 * np.pi ) - np.pi
	return angle

def calc_angle(robot, goal):
	angle2Goal = math.atan2(goal.y-robot.pos.y, goal.x-robot.pos.x)
	angle2Goal = wrap2Pi(robot.ori.theta - angle2Goal)
	return angle2Goal

def calc_dist2Goal(robot, goal):
	distFromGoal = math.sqrt((goal.x-robot.pos.x)**2 + (goal.y-robot.pos.y)**2)
	return distFromGoal	

def simple_cmd(robot, goal):
	cmd = command(0,0)
	angle_diff = calc_angle(robot, goal)
	angle_epsilon = 1.5*math.pi/180
	reverse = 0 # 1 = normal, 0 = reverse
	distFromGoal = calc_dist2Goal(robot, goal)
	dist_epsilon = 0.01

	if abs(angle_diff) > math.pi/2:
		reverse = 1
		angle_diff = wrap2Pi(angle_diff + math.pi) 

	if abs(angle_diff) < robot.get_minStopAngle():
		cmd.ang = -robot.get_maximumAngleVel(angle_diff) #robot.vel.ang
	elif abs(angle_diff) > robot.get_minStopAngle():
		cmd.ang = -robot.get_maximumAngleVel(angle_diff)
	
	if abs(cmd.ang) < angle_epsilon*2:
		if distFromGoal < robot.get_minStopDistance() + dist_epsilon:
			cmd.lin = -robot.get_maxStopVelocity()
		else:
			cmd.lin = robot.lim.v
		if reverse == 1:
			cmd.lin *= -1
	return cmd

def generate_response(target):
	if latched_rob_pos == 0:
		print("Robot position is not yet available.")
		return
	print ("Target coordinates: x=%.2f y=%.2f, planning time resolution: %f"%(target.x, target.y, target.time_res))
	# Setup
	# TODO robot position from topic
	robot = robot_instance()
	robot.pos.x = robot_state.x_pos
	robot.pos.y = robot_state.y_pos
	robot.ori.theta = wrap2Pi(robot_state.eul_pos)
	goal = point_instance(target.x, target.y)
	cmd = command(0,0)
	timeInterval = target.time_res	 # 1 sec division
	timeInterval = 1/timeInterval
	dist_epsilon = 0.01 #m
	vel_epsilon = 0.01 # m/s
	distFromGoal = calc_dist2Goal(robot, goal)
	it = 1
	it_limit = int(1/timeInterval * distFromGoal * 100) #Might need to be changed
	# Return arrays
	response = list()
	lin_velocity_list = []
	ang_velocity_list = []

	print("Distance from goal: %.2f"%(distFromGoal))
	while ((distFromGoal >= dist_epsilon and it < it_limit) or (distFromGoal < dist_epsilon and abs(robot.vel.lin) > vel_epsilon and it < it_limit)):
		cmd = simple_cmd(robot, goal)
		robot.move(cmd, timeInterval)
		distFromGoal = calc_dist2Goal(robot, goal)
		lin_velocity_list.append(robot.vel.lin)
		ang_velocity_list.append(robot.vel.ang)
		it += 1
	print("Distance from goal %.2f, robot speed = %.2f, robot angular velocity = %.2f, robot ori: %.2f" % (distFromGoal,robot.vel.lin,robot.vel.ang, robot.ori.theta))
	print("Location %.2f, %.2f" %(robot.pos.x, robot.pos.y))
	print("Number of iterations: %i, iteration limit: %i" % (it, it_limit))
	if it == it_limit:
		print("Iteration limit reached, empty response")
	else:
		print("Returning")
		response.append(lin_velocity_list)
		response.append(ang_velocity_list)
	return response

def robot_state_callback(msg):
	robot_state.x_pos = msg.x_pos
	robot_state.y_pos = msg.y_pos
	robot_state.eul_pos = msg.eul_pos
	global latched_rob_pos
	if latched_rob_pos == 0: 
		latched_rob_pos = 1

def robot_state_callback_odom(data):
	eul = tf.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
					data.pose.pose.orientation.z, data.pose.pose.orientation.w])
	robot_state.x_pos = data.pose.pose.position.x
	robot_state.y_pos = data.pose.pose.position.y
	robot_state.eul_pos = eul[2]
	global latched_rob_pos
	if latched_rob_pos == 0: 
		latched_rob_pos = 1

def planner_server():
	rospy.init_node('planner_server')
	rate = rospy.Rate(1000) # 1000hz
	rospy.Subscriber('/robotPos', robot_msg, robot_state_callback)
	rospy.Subscriber('/odom',Odometry, robot_state_callback_odom)
	s = rospy.Service('planner_service', planner_srv, generate_response)
	print("Planner service is running.")
	rospy.spin()


if __name__ == "__main__":
	global robot_state, latched_rob_pos
	robot_state = robot_msg()
	latched_rob_pos = 0
	try:
		while not rospy.is_shutdown():
			planner_server()
	except rospy.ROSInterruptException:
		pass