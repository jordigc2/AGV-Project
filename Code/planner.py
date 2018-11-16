import math
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


class robot_instance:
	def __init__(self):
		self.pos = self.position(0,0)
		self.ori = self.orientation(0)
		self.vel = self.velocity()
		self.lim = self.limits(1.0, 0.2, 1.2, 0.4)

	class position:
		def __init__(self, a, b):
			self.x = a
			self.y = b
	class orientation:
		def __init__(self,a):
			self.theta = a
	class velocity:
		def __init__(self):
			self.x = 0
			self.ang = 0
	class limits:
		def __init__(self,a,b,c,d):
			self.v = a
			self.a = b
			self.w = c
			self.w_dot = d

class point_instance:
	def __init__(self, x_p, y_p):
		self.x = x_p
		self.y = y_p

class command():
	def __init__(self,a,b):
		self.vel = a
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

def move_robot(robot, command, timeInterval):
	robot.ori.theta += command.ang / timeInterval
	robot.ori.theta = wrap2Pi(robot.ori.theta)
	robot.vel.x = command.vel
	robot.pos.x += robot.vel.x * math.cos(robot.ori.theta) / timeInterval
	robot.pos.y += robot.vel.x * math.sin(robot.ori.theta) / timeInterval
	return robot

def next_pos(robot):
	lowest_dist = 100000
	i = 0
	for i in range(-1, 2, 2):
		rob_next = robot_instance()
		rob_next.pos.x += (robot.vel.x + i * robot.lim.a / timeInterval) * math.cos(robot.ori.theta) / timeInterval
		rob_next.pos.y += (robot.vel.x + i * robot.lim.a / timeInterval) * math.sin(robot.ori.theta) / timeInterval
		next_dist = calc_dist2Goal(rob_next, goal)
		if next_dist < lowest_dist:
			lowest_dist = next_dist
			best_operator = i
	return best_operator

def select_vel(robot, resolution, timeLookahead):
	lowest_dist = 100000
	if timeInterval/timeLookahead>100:
		print("WARNING! Planner lookahead - timeInterval Ratio is too high which may lead to instabilities!")
	if resolution%2 !=1:
		resolution += 1
	if resolution < 3:
		resolution = 3
	for i in range(int((resolution+1)/2-resolution), int(resolution-(resolution-1)/2) ,1):
		rob_next = robot_instance()
		next_vel = robot.vel.x + i/((resolution-1)/2) * robot.lim.a * timeLookahead
		if abs(next_vel) > robot.lim.v:
			next_vel = math.copysign(robot.lim.v, next_vel)

		rob_next.pos.x += next_vel * math.cos(robot.ori.theta)
		rob_next.pos.y += next_vel * math.sin(robot.ori.theta)
		next_dist = calc_dist2Goal(rob_next, goal)
		if next_dist < lowest_dist:
			lowest_dist = next_dist
			best_vel = next_vel
	#Correct for time
	best_vel = best_vel/timeLookahead
	return best_vel

def generate_command(robot, goal, timeInterval, dist_epsilon):
	robot_vel = robot.vel.x
	cmd = command(robot_vel, robot.vel.ang)
	goalAngle = calc_angle(robot,goal)
	## If turn > 90 then reverse direction
	if abs(goalAngle)>math.pi/2:
		robot.ori.theta = wrap2Pi(robot.ori.theta + math.pi)
		goalAngle = calc_angle(robot,goal)

	## Linear velocity targets
	stopTime = robot_vel/robot.lim.a
	if stopTime == 0:
		stopDistance = 0
	else:
		stopDistance = robot_vel * stopTime / 2
	distFromGoal = calc_dist2Goal(robot,goal)
	stopDistance_map.append(stopDistance)
	if stopTime != 0:
		stopVel = distFromGoal / stopTime
	else:
		stopVel = robot.lim.v
	stopvel_map.append(stopVel)

	if distFromGoal <= stopDistance + dist_epsilon:
		cmd.vel -= (math.copysign(robot.lim.a, cmd.vel) / timeInterval)
	else: 
		## Predict future state, chose to minimize distance to goal
		#cmd.vel += math.copysign(robot.lim.a, next_pos(robot)) / timeInterval
		cmd.vel = select_vel(robot, 9, 1)
	if abs(cmd.vel) > abs(stopVel):
		cmd.vel = math.copysign(stopVel, cmd.vel) 
	if abs(cmd.vel) > robot.lim.v:
		cmd.vel = math.copysign(robot.lim.v, cmd.vel)

	##BUG BUG BUG

	## Angular velocity
	if cmd.ang - goalAngle > robot.lim.w_dot / timeInterval:
		cmd.ang += math.copysign(robot.lim.w_dot/timeInterval, goalAngle)
	else:
		cmd.ang -= goalAngle
	if abs(cmd.ang) > robot.lim.w:
		cmd.ang = math.copysign(robot.lim.w, cmd.ang)
	print("Start angle: %3f, angle difference: %3f, angle cmd: %3f" %(robot.ori.theta, goalAngle, cmd.ang))
	return cmd

	##BUG BUG BUG



print("****Start****")
#Variables
robot = robot_instance()
goal = point_instance(-3,1)
goal_array = [goal, point_instance(-3,-2), point_instance(1,1), point_instance(2,-2), point_instance(0,0)]
cmd = command(0,0)
timeInterval = 10	 # 1 sec division
dist_epsilon = 0.05 #cm
vel_epsilon = 0.01 # m/s
distFromGoal = calc_dist2Goal(robot, goal)
it = 1
it_limit = int(timeInterval * distFromGoal * 7)

#Figures
velocity_map = []
ang_map = []
stopvel_map = []
distance_map = []
stopDistance_map = []
pos_x_map = []
pos_y_map = []

for i in range(len(goal_array)-1):
	print(i)
	goal = goal_array[i]
	distFromGoal = calc_dist2Goal(robot, goal)
	it = 1
	it_limit = int(timeInterval * distFromGoal * 7)
	print("Distance from goal = %f" %distFromGoal)
	while ((distFromGoal >= dist_epsilon and it < it_limit) or (distFromGoal < dist_epsilon and robot.vel.x > vel_epsilon and it < it_limit)):
		cmd = generate_command(robot, goal, timeInterval, dist_epsilon)
		robot = move_robot(robot, cmd, timeInterval)
		distFromGoal = calc_dist2Goal(robot, goal)
		if i == 3:
			velocity_map.append(robot.vel.x)
			ang_map.append(cmd.ang)
			distance_map.append(distFromGoal)
			pos_x_map.append(robot.pos.x)
			pos_y_map.append(robot.pos.y)
		it += 1
	print("Distance from goal %.2f, robot speed = %.2f" % (distFromGoal,robot.vel.x))
	print("Location %f, %f" %(robot.pos.x, robot.pos.y))
	print("Number of iterations: %i, iteration limit: %i" % (it, it_limit))
	if it == it_limit:
		print("Iteration limit reached")
print("***End***")


## Ploting
fig, ax = plt.subplots()
x = np.arange(0, len(velocity_map), 1)
ax.plot(x, velocity_map)
ax.plot(x, distance_map)
ax.plot(x, ang_map)
#ax.plot(x, stopDistance_map)
#ax.plot(x, stopvel_map)
ax.set(xlabel='iteration', ylabel='Value',
       title='Velocity and distance in each iteration')
ax.grid()   
plt.show()

## Ploting
fig2, bx = plt.subplots()
bx.plot(pos_x_map, pos_y_map)
bx.set(xlabel='x', ylabel='y',
       title='Path')
bx.grid()   
plt.show()














# Code graveyard
# elif robot_vel<=robot.lim.v:
# 		if robot.lim.v-robot_vel > robot.lim.a / timeInterval:
# 			cmd.vel = robot_vel + math.copysign(robot.lim.a, next_pos(robot)) / timeInterval
# 		else:
# 			cmd.vel = math.copysign(robot.lim.v, cmd.vel)
