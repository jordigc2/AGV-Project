#!/usr/bin/env python
import rospy
from planner_pkg.srv import *
import math
import numpy as np
from planner_pkg.msg import robot as robot_msg
from planner_pkg.msg import target as target_msg
from tf import transformations as tf
import geometry_msgs.msg as gmsgs
from nav_msgs.msg import Odometry


class robot_instance:
	def __init__(self):
		self.pos = self.position(0,0)
		self.ori = self.orientation(0)
		self.vel = self.velocity()
		self.lim = self.limits(0.5, 2.0, 7.0, 38.0)#REal tests crude  #Sim gentle. works better(2.0, 2.5, 7.0, 3.5) #Sim(2.5, 3.33, 9.2, 5.71)  #Set these according to robot constraintes

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
	def __init__(self, x_p, y_p, theta_p):
		self.x = x_p
		self.y = y_p
		self.ori = theta_p

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
		cmd.ang = -robot.get_maximumAngleVel(angle_diff)
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

def run_node(pub):
	if latched_rob_pos == 0:
		print("Robot position is not yet available.")
		return
	# Setup
	cmd = command(0,0)
	dist_epsilon = 0.01 #m
	vel_epsilon = 0.01 # m/s
	distFromGoal = calc_dist2Goal(robot, goal)

	if ((distFromGoal >= dist_epsilon) or (distFromGoal < dist_epsilon and (abs(robot.vel.lin) > vel_epsilon) or abs(robot.vel.ang) > vel_epsilon)):
		cmd = simple_cmd(robot, goal)
		print(cmd.lin,cmd.ang)
		# Hopefully we switch to only using one version of the object
		cmd_format = target_msg()
		cmd_format.linvel = cmd.lin
		cmd_format.angvel = cmd.ang
		pub.publish(cmd_format)

def robot_state_callback_TS(data):
	eul = tf.euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, 
					data.transform.rotation.z, data.transform.rotation.w])
	robot.pos.x = data.transform.translation.x
	robot.pos.y = data.transform.translation.y
	robot.ori.theta = eul[2]
	global latched_rob_pos
	if latched_rob_pos == 0: 
		latched_rob_pos = 1

def goal_callback(msg):
	goal.x = msg.x_pos
	goal.y = msg.y_pos
	goal.ori = msg.eul_pos # 0 if waypoint

def robot_velocity_callback(msg):
	robot.vel.lin = msg.linvel
	robot.vel.ang = msg.angvel

def node_setup():
	rospy.init_node('robot_control')
	rospy.Subscriber('/vicon/AGVgr5/AGVgr5', gmsgs.TransformStamped, robot_state_callback_TS)
	rospy.Subscriber('/vicon/SomethingIdunnuWhatever/SomethingIdunnuWhatever', gmsgs.TransformStamped, robot_state_callback_TS)
	rospy.Subscriber("/goalPos", robot_msg, goal_callback)
	rospy.Subscriber("/arduino/vel", target_msg, robot_velocity_callback)
	vel_pub = rospy.Publisher("/some/topic", target_msg, queue_size=1)
	print("Robot control is running.")
	return vel_pub

if __name__ == "__main__":
	global robot, latched_rob_pos, goal
	robot = robot_instance()
	goal = point_instance(0,0,0)
	latched_rob_pos = 0
	try:
		pub = node_setup()
		rate = rospy.Rate(1000) # 1000hz
		while not rospy.is_shutdown():
			run_node(pub)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass