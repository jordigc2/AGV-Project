import math
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy


class robot_instance:
	def __init__(self):
		self.pos = self.position(0,0)
		self.ori = self.orientation(0)
		self.vel = self.velocity()
		self.lim = self.limits(1.0, 1.0, 1.8, 1.0)

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

	def __deepcopy__(self, memo):
		cls = self.__class__
		result = cls.__new__(cls)
		memo[id(self)] = result
		for k, v in self.__dict__.items():
			setattr(result, k, deepcopy(v, memo))
		return result

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

def select_next_cmd(robot, resolution, timeLookahead):
	## Predict future state, chose to minimize distance to goal
	lowest_dist = 100000
	lowest_angle = 100000
	best_cmd = command(0,0)
	if 1/timeInterval/timeLookahead>100:
		print("WARNING! Planner lookahead - timeInterval Ratio is too high which may lead to instabilities!")
	if resolution%2 !=1:
		resolution += 1
	if resolution < 3:
		resolution = 3
	for i in range(int((resolution+1)/2-resolution), int(resolution-(resolution-1)/2) ,1):
		for j in range(int((resolution+1)/2-resolution), int(resolution-(resolution-1)/2) ,1):
			rob_next = deepcopy(robot)
			next_cmd = command(0,0)
			next_cmd.lin = robot.lim.v * i/((resolution-1)/2)
			next_cmd.ang = robot.lim.w * j/((resolution-1)/2)
			rob_next.move(next_cmd, timeLookahead)
			next_dist = calc_dist2Goal(rob_next, goal)
			next_angle = calc_angle(rob_next, goal)
			ori_diff = 0 - rob_next.ori.theta
			if math.sqrt(next_dist**2 + next_angle**2) <= math.sqrt(lowest_dist**2 + lowest_angle**2):
				lowest_dist = next_dist
				lowest_angle = next_angle
				best_cmd.lin = next_cmd.lin
				best_cmd.ang = next_cmd.ang
	return best_cmd

def simple_cmd(robot, goal):
	cmd = command(0,0)
	angle_diff = calc_angle(robot, goal)
	angle_epsilon = 1.5*math.pi/180
	reverse = 0 # 1 = normal, 0 = reverse

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

####################################################################

print("****Start****")
#Variables
robot = robot_instance()
goal = point_instance(2,0)
#goal_array = [goal, point_instance(1, 0), point_instance(0.7,0.5), point_instance(1,0), point_instance(2,0)]
cmd = command(0,0)
timeInterval = 100	 # 1 sec division
timeInterval = 1/timeInterval
dist_epsilon = 0.01 #m
vel_epsilon = 0.01 # m/s
distFromGoal = calc_dist2Goal(robot, goal)
it = 1
it_limit = int(timeInterval * distFromGoal * 15)

#Variables for Figures
velocity_map = [0]
ang_map = [0]
stopvel_map = [0]
distance_map = [distFromGoal]
stopDistance_map = [0]
pos_x_map = [0]
pos_y_map = [0]
lookahead_map = [1]
cmd_map = [0]

for i in range(len(goal_array)):
	print(i)
	goal = goal_array[i]
	distFromGoal = calc_dist2Goal(robot, goal)
	it = 1
	it_limit = int(1/timeInterval * distFromGoal * 100)
	print("Distance from goal = %f" %distFromGoal)

	while ((distFromGoal >= dist_epsilon and it < it_limit) or (distFromGoal < dist_epsilon and abs(robot.vel.lin) > vel_epsilon and it < it_limit)):
		stopTime = robot.get_minStopTime()
		#cmd = select_next_cmd(robot, 19, 1)
		cmd = simple_cmd(robot, goal)
		robot.move(cmd, timeInterval)
		distFromGoal = calc_dist2Goal(robot, goal)
		#For figures
		velocity_map.append(robot.vel.lin)
		ang_map.append(robot.ori.theta)
		distance_map.append(distFromGoal)
		stopvel_map.append(robot.get_maxStopVelocity())
		cmd_map.append(cmd.lin)
		pos_x_map.append(robot.pos.x)
		pos_y_map.append(robot.pos.y)
		it += 1
	print((distFromGoal >= dist_epsilon and it < it_limit), (distFromGoal < dist_epsilon and robot.vel.lin > vel_epsilon and it < it_limit))
	print("Distance from goal %.2f, robot speed = %.2f, robot angular velocity = %.2f, robot ori: %.2f" % (distFromGoal,robot.vel.lin,robot.vel.ang, robot.ori.theta))
	print("Location %.2f, %.2f" %(robot.pos.x, robot.pos.y))
	print("Number of iterations: %i, iteration limit: %i" % (it, it_limit))
	if it == it_limit:
		print("Iteration limit reached")
print("***End***")


## Ploting
fig, ax = plt.subplots()
x = np.arange(0, len(velocity_map), 1)
#ax.plot(x, cmd_map)
ax.plot(x, velocity_map)
ax.plot(x, distance_map)
ax.plot(x, ang_map)
#ax.plot(x, stopvel_map)
ax.set(xlabel='iteration', ylabel='Value',
	   title='Velocity and distance in each iteration')
ax.grid()   

## Ploting
fig2, bx = plt.subplots()
bx.plot(pos_x_map, pos_y_map)
bx.set(xlabel='x', ylabel='y',
	   title='Path')
bx.grid()   
plt.show()













### Code graveyard ###

# elif robot_vel<=robot.lim.v:
# 		if robot.lim.v-robot_vel > robot.lim.a / timeInterval:
# 			cmd.vel = robot_vel + math.copysign(robot.lim.a, next_pos(robot)) / timeInterval
# 		else:
# 			cmd.vel = math.copysign(robot.lim.v, cmd.vel)

# def next_pos(robot):
# 	lowest_dist = 100000
# 	i = 0
# 	for i in range(-1, 2, 2):
# 		rob_next = robot_instance()
# 		rob_next.pos.x += (robot.vel.lin + i * robot.lim.a / timeInterval) * math.cos(robot.ori.theta) / timeInterval
# 		rob_next.pos.y += (robot.vel.lin + i * robot.lim.a / timeInterval) * math.sin(robot.ori.theta) / timeInterval
# 		next_dist = calc_dist2Goal(rob_next, goal)
# 		if next_dist < lowest_dist:
# 			lowest_dist = next_dist
# 			best_operator = i
# 	return best_operator

# if robot.ori.theta - goalAngle > robot.lim.w / timeInterval:
# 	cmd.ang += math.copysign(robot.lim.w_dot/timeInterval, goalAngle)
# else:
# 	cmd.ang -= goalAngle

## If turn > 90 then reverse direction
#	if abs(goalAngle)>math.pi/2:
#		robot.ori.theta = wrap2Pi(robot.ori.theta + math.pi)
#		goalAngle = calc_angle(robot,goal)

# def select_vel(robot, resolution, timeLookahead):
# 	lowest_dist = 100000
# 	if timeInterval/timeLookahead>100:
# 		print("WARNING! Planner lookahead - timeInterval Ratio is too high which may lead to instabilities!")
# 	if resolution%2 !=1:
# 		resolution += 1
# 	if resolution < 3:
# 		resolution = 3
# 	for i in range(int((resolution+1)/2-resolution), int(resolution-(resolution-1)/2) ,1):
# 		rob_next = robot_instance()
# 		next_vel = robot.vel.lin + i/((resolution-1)/2) * robot.lim.a
# 		if abs(next_vel) > robot.lim.v:
# 			next_vel = math.copysign(robot.lim.v, next_vel)

# 		rob_next.pos.x += next_vel * math.cos(robot.ori.theta) * timeLookahead
# 		rob_next.pos.y += next_vel * math.sin(robot.ori.theta) * timeLookahead
# 		next_dist = calc_dist2Goal(rob_next, goal)
# 		if next_dist < lowest_dist:
# 			lowest_dist = next_dist
# 			best_vel = next_vel
# 	#Correct for time
# 	best_vel = best_vel/timeLookahead
# 	return best_vel

# def generate_command(robot, goal, timeInterval, dist_epsilon):
# 	cmd = command(robot.vel.lin, robot.vel.ang)
# 	tmp_cmd = command(0,0)
# 	goalAngle = calc_angle(robot,goal)

# 	## Stopping criteria
# 	stopTime = robot.vel.lin/robot.lim.a
# 	#stopDistance = 0 if robot.vel.lin == 0 else robot.vel.lin * stopTime / 2
# 	stopDistance = robot.vel.lin * stopTime / 2
# 	distFromGoal = calc_dist2Goal(robot,goal)
# 	stopVel = distFromGoal / stopTime if stopTime != 0 else robot.lim.v
# 	#For figures
# 	stopDistance_map.append(stopDistance)
# 	stopvel_map.append(stopVel)

# 	# ## When close to goal according to stop limits
# 	# if distFromGoal <= stopDistance + dist_epsilon:
# 	# 	cmd.vel -= (math.copysign(robot.lim.a, cmd.vel) / timeInterval)
# 	# else: 
# 	# 	tmp_cmd = select_next_cmd(robot, 9, 1)
# 	# 	cmd.vel = tmp_cmd.vel
# 	# if abs(cmd.vel) > abs(stopVel):
# 	# 	cmd.vel = math.copysign(stopVel, cmd.vel) 
# 	# if abs(cmd.vel) > robot.lim.v:
# 	# 	cmd.vel = math.copysign(robot.lim.v, cmd.vel)


# 	# ## Angular velocity
# 	# if distFromGoal <= stopDistance + dist_epsilon:
# 	# 	if abs(cmd.ang) > robot.lim.w_dot / timeInterval:
# 	# 		cmd.ang -= (math.copysign(robot.lim.w_dot, cmd.ang) / timeInterval)
# 	# 	else:
# 	# 		cmd.ang -= (math.copysign(cmd.ang, cmd.ang))
# 	# else: 
# 	# 	cmd.ang = tmp_cmd.ang

# 	# if abs(cmd.ang) > robot.lim.w:
# 	# 	cmd.ang = math.copysign(robot.lim.w, cmd.ang)
# 	return cmd

# def move_robot(robot, command, timeInterval):
# 	robot.vel.ang = command.ang
# 	robot.ori.theta += robot.vel.ang / timeInterval
# 	robot.ori.theta = wrap2Pi(robot.ori.theta)
# 	robot.vel.lin = command.lin
# 	robot.pos.x += robot.vel.lin * math.cos(robot.ori.theta) / timeInterval
# 	robot.pos.y += robot.vel.lin * math.sin(robot.ori.theta) / timeInterval
# 	return robot

			# Local Dynamic window approach
			#next_cmd.lin = robot.vel.lin + i/((resolution-1)/2) * robot.lim.a * timeLookahead
			#next_cmd.ang = robot.vel.ang + j/((resolution-1)/2) * robot.lim.w_dot * timeLookahead
			#if abs(next_cmd.lin) > robot.lim.v:
			#	next_cmd.lin = math.copysign(robot.lim.v, next_cmd.lin)
			#if abs(next_cmd.ang) > robot.lim.w:
			#	next_cmd.ang = math.copysign(robot.lim.w, next_cmd.ang)