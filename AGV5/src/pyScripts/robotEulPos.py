#!/usr/bin/env python
import rospy
from tf import transformations as tf
import geometry_msgs.msg as gmsgs
import sys
from AGV5.msg import robot


def pubEulPos(data):
	eul = tf.euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, 
					data.transform.rotation.z, data.transform.rotation.w])
	#sys.stdout.write("\rX_trans,Y_trans: "+str(data.transform.translation.x)+", " +
	#str(data.transform.translation.y) + " eul: "+str(eul[2]))
	pos = robot()
	pos.x_pos = data.transform.translation.x
	pos.y_pos = data.transform.translation.y
	pos.eul_pos = eul[2]
	posPub.publish(pos)

if __name__ == '__main__':
	print "Reading from Vicon"
	rospy.init_node('VICON_reader')
	rospy.Subscriber('/vicon/AGVgr5/AGVgr5',gmsgs.TransformStamped, pubEulPos)
	posPub = rospy.Publisher('/robotPos', robot, queue_size=10)
	rospy.spin()
