#!/usr/bin/env python

import rospy
import geometry_msgs.msg as gmsgs
from tf import transformations as tf
import sys

def printCB(data):
	eul = tf.euler_from_quaternion([data.transform.rotation.x, data.transform.rotation.y, 
					data.transform.rotation.z, data.transform.rotation.w])
	sys.stdout.write("\rX_traans,Y_trans:",data.transform.translation.x, 
			data.transform.translation.y,"eul:",eul[2])
	#print("X_traans,Y_trans:",data.transform.translation.x, data.transform.translation.y)
	#print("eul:",eul[2])

def readFromVICON():
	rospy.init_node('VICON_reader', anonymous=True)
	rospy.Subscriber('/vicon/as2018_test/as2018_test',gmsgs.TransformStamped, printCB)
	rospy.spin()

if __name__ == '__main__':
	print("_Reading from VICON topic_")
	readFromVICON()
