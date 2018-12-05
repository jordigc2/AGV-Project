#!/usr/bin/env python

import rospy
from tf import transformations as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from planner_pkg.msg import robot as robot_msg
from planner_pkg.msg import target as target_msg
from planner_pkg.srv import *
import math

#Don't hate on me for the globals pls

def goal_callback(msg):
	goal.x_pos = msg.x_pos
	goal.y_pos = msg.y_pos
	goal.eul_pos = msg.eul_pos # 0 if waypoint
	global new_goal
	new_goal = 1

def robot_state_callback_odom(data):
	eul = tf.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
					data.pose.pose.orientation.z, data.pose.pose.orientation.w])
	robot_state.x_pos = data.pose.pose.position.x
	robot_state.y_pos = data.pose.pose.position.y
	robot_state.eul_pos = eul[2]
	global latched_rob_pos
	if latched_rob_pos == 0: 
		latched_rob_pos = 1

def robot_state_callback(msg):
	robot_state.x_pos = msg.x_pos
	robot_state.y_pos = msg.y_pos
	robot_state.eul_pos = msg.eul_pos
	global latched_rob_pos
	if latched_rob_pos == 0: 
		latched_rob_pos = 1

def node_setup():
	# Change returned 
	pub_list = []
	rospy.init_node("robot_guidance")
	# Physical robot control
	rospy.Subscriber("/robotPos", robot_msg, robot_state_callback)
	rospy.Subscriber("/goalPos", robot_msg, goal_callback)
	vel_pub = rospy.Publisher("/some/topic", target_msg, queue_size=10)
	# Simulation control
	rospy.Subscriber('/odom',Odometry, robot_state_callback_odom)
	sim_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub_list.extend([vel_pub, sim_vel_pub])
	return pub_list

def calc_dist2Goal(robot, goal):
	distFromGoal = math.sqrt((goal.x_pos-robot.x_pos)**2 + (goal.y_pos-robot.y_pos)**2)
	return distFromGoal	

def run_node(pub_list):
	global linvel, angvel
	global start_time
	time_resolution = 100 # 100 Hz

	global new_goal, latched_rob_pos
	if latched_rob_pos == 0:
		return
	if (len(linvel) == 0 or len(angvel) == 0) and new_goal == 1:
		print("New goal received x=%.2f, y=%.2f"%(goal.x_pos, goal.y_pos))
		print(new_goal)
		new_goal = 0
		#rospy.wait_for_service('planner_server')
		try:
			planner_service = rospy.ServiceProxy('planner_service', planner_srv)
			response = planner_service(goal.x_pos, goal.y_pos, time_resolution)
			linvel = list(response.v)
			angvel = list(response.omega)
			start_time = rospy.Time.now()
			#print("start time: %f"%start_time.to_sec())
			#print("planner size %i"%(len(linvel)))
		except rospy.ServiceException, e:
			print ("Service call failed: %s"%e)

	# Default message
	msg = target_msg()
	msg.linvel = 0
	msg.angvel = 0
	# Pop if elapsed time > time_resolution
	now = rospy.Time.now()
	if not len(linvel) == 0 or not len(angvel) == 0:
		if now - start_time >= rospy.Duration(1.0/time_resolution): 
			start_time = now
			linvel.pop(0)
			angvel.pop(0)
		# If close to goal stop, else motion
		if calc_dist2Goal(robot_state, goal) > 0.02 and not len(linvel) == 0 and not len(angvel) == 0:
			msg.linvel = linvel[0] * 0.95 # .95 to reduce overshoot because of simualation update frequency delay
			msg.angvel = angvel[0] * 0.95
			if msg.angvel < 0.02:
				msg.angvel = 0
	pub_list[0].publish(msg)
	# Simulation velocity publisher
	sim_msg = Twist()
	sim_msg.linear.x = msg.linvel
	sim_msg.angular.z = msg.angvel
	pub_list[1].publish(sim_msg)


if __name__ == "__main__":
	print("Robot guidance is running.")
	print("Waiting for goal positions.")
	#rospy.sleep(2)
	global robot_state, goal, new_goal, latched_rob_pos
	new_goal = 0
	latched_rob_pos = 0
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