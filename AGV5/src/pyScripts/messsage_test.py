#!/usr/bin/env python

import rospy
from AGV5.msg import target

pub = rospy.Publisher('target/velocities', target, queue_size=1000)

rospy.init_node('publish_mymessage')
r = rospy.Rate(2)

while not rospy.is_shutdown():

	t = target()

	t.linvel = 0.8
	t.angvel = 0.4

	pub.publish(t)
	r.sleep()
