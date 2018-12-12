#!/usr/bin/env python

import rospy
from tf import transformations as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from planner_pkg.msg import robot as robot_msg
from planner_pkg.msg import target as target_msg
from planner_pkg.srv import *
import math

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


def robot_state_callback_odom(data):
	global lin_vel, ang_vel
	vel = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)
	lin_vel.append(vel)
	ang_vel.append(data.twist.twist.angular.z)
	pos_x.append(data.pose.pose.position.x)
	pos_y.append(data.pose.pose.position.y)
	global latched
	if latched == 0:
		latched = 1

def robot_state_callback_TS(data):
	global lin_vel, ang_vel
	pos_x.append(data.transform.translation.x)
	pos_y.append(data.transform.translation.y)
	global latched
	if latched == 0:
		latched = 1

def velocity_command_callback(data):
	global cmd_lin, cmd_ang
	vel = math.sqrt(data.linear.x**2 + data.linear.y**2)
	cmd_lin.append(vel)
	cmd_ang.append(data.angular.z)

def node_setup():
	# Change returned 
	pub_list = []
	rospy.init_node("robot_limits")
	# Physical robot control
	rospy.Subscriber('/vicon/trial_object/trial_object',TransformStamped, robot_state_callback_TS)
	vel_pub = rospy.Publisher("/some/topic", target_msg, queue_size=10)
	# Simulation control
	rospy.Subscriber('/odom',Odometry, robot_state_callback_odom)
	rospy.Subscriber('/cmd_vel',Twist, velocity_command_callback)
	sim_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub_list.extend([vel_pub, sim_vel_pub])
	return pub_list

def run_node(pub_list, lin, ang):
	sim_msg = Twist()
	sim_msg.linear.x = lin
	sim_msg.angular.z = ang
	vel_msg = target_msg()
	vel_msg.linvel = lin
	vel_msg.angvel = ang
	global latched
	if latched == 1:
		pub_list[0].publish(vel_msg)
		pub_list[1].publish(sim_msg)


if __name__ == "__main__":
	print("Robot limit plots")
	global lin_vel, ang_vel, latched, cmd_lin, cmd_ang, pos_x, pos_y
	latched = 0 
	lin_vel = []
	ang_vel = []
	cmd_lin = []
	cmd_ang = []
	pos_x = []
	pos_y = []
	try:
		pub = node_setup()
		start_time = rospy.Time.now()
		now = start_time
		while not rospy.is_shutdown() and now - start_time < rospy.Duration(2):
			run_node(pub,2.5,0)
			now = rospy.Time.now()
			rate = rospy.Rate(100)
			rate.sleep()
		run_node(pub,0,0)
	except rospy.ROSInterruptException:
		pass

	
	if not len(lin_vel) == 0:
		fig, ax = plt.subplots()
		ang_plot = ang_vel
		lin_plot = lin_vel
		x = np.arange(0, len(ang_plot), 1)
		ax.plot(x, ang_plot)
		xx = np.arange(0, len(lin_plot), 1)
		ax.plot(xx, lin_plot)
		ax.set(xlabel='iteration', ylabel='Value',
			   title='Velocity m/s')
		ax.grid()   
		fig.savefig("Robot velocities.png")

	if not len(cmd_lin) == 0:
		fig, bq = plt.subplots()
		cmd_lin_plot = cmd_lin
		cmd_ang_plot = cmd_ang
		q = np.arange(0, len(cmd_ang_plot), 1)
		bq.plot(q, cmd_ang_plot)
		qq = np.arange(0, len(cmd_lin_plot), 1)
		bq.plot(qq, cmd_lin_plot)
		bq.set(xlabel='iteration', ylabel='Value',
			   title='Velocity commands m/s')
		bq.grid()
		fig.savefig("Velocity commands.png")

	if not len(pos_x) == 0:
		fig, cr = plt.subplots()
		plot_x = pos_x
		plot_y = pos_y
		if len(plot_x) > len(plot_y):
			del(plot_x[len(plot_y)-1:])
		elif len(plot_y) > len(plot_x):
			del(plot_y[len(plot_x)-1:])
		cr.plot(plot_x, plot_y)
		cr.set(xlabel='X (m)', ylabel='Y (m)',
			   title='Robot path')
		cr.grid()
		fig.savefig("Robot path.png")
	
	plt.show()