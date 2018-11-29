#!/usr/bin/env python

import rospy
from planner_pkg.msg import robot as robot_msg
from planner_pkg.msg import target as target_msg
from planner_pkg.srv import *
import math


def goal_callback(msg):
	goal.x_pos = msg.x_pos
	goal.y_pos = msg.y_pos
	goal.eul_pos = msg.eul_pos # 0 if waypoint

def robot_state_callback(msg):
	robot_state.x_pos = msg.x_pos
	robot_state.y_pos = msg.y_pos
	robot_state.eul_pos = msg.eul_pos

def node_setup():
	rospy.init_node("robot_guidance")
	rospy.Subscriber("robotPos", robot_msg, robot_state_callback)
	rospy.Subscriber("goalPos", robot_msg, goal_callback)
	vel_pub = rospy.Publisher("/some/topic", target_msg, queue_size=100)
	return vel_pub

def calc_dist2Goal(robot, goal):
	distFromGoal = math.sqrt((goal.x_pos-robot.x_pos)**2 + (goal.y_pos-robot.y_pos)**2)
	return distFromGoal	

def run_node(vel_pub):
	global linvel, angvel
	global start_time
	time_resolution = 100 # 100 Hz

	if len(linvel) == 0 or len(angvel) == 0:
		#rospy.wait_for_service('planner_server')
		try:
			planner_service = rospy.ServiceProxy('planner_service', planner_srv)
			response = planner_service(goal.x_pos, goal.y_pos, time_resolution)
			linvel = list(response.v)
			angvel = list(response.omega)
			start_time = rospy.Time.now()
		except rospy.ServiceException, e:
			print ("Service call failed: %s"%e)

	# Default message
	msg = target_msg()
	msg.linvel = 0
	msg.angvel = 0
	# Pop if elapsed time > time_resolution
	now = rospy.Time.now()
	if now - start_time >= rospy.Duration(1/time_resolution):
		if not len(linvel) == 0 or not len(angvel) == 0:
			start_time = start_time + rospy.Duration(1/time_resolution)
			print(angvel)
			linvel.pop(0)
			angvel.pop(0)
	# If close to goal stop, else motion
	if calc_dist2Goal(robot_state, goal) > 0.05 and not len(linvel) == 0 and not len(angvel) == 0:
		msg.linvel = linvel[0]
		msg.angvel = angvel[0]
	vel_pub.publish(msg)

if __name__ == "__main__":
	print("Robot guidance is running")
	#rospy.sleep(2)
	global robot_state, goal
	linvel = []
	angvel = []
	robot_state = robot_msg()
	goal = robot_msg()
	try:
		pub = node_setup()
		while not rospy.is_shutdown():
			run_node(pub)
			rate = rospy.Rate(1000)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass