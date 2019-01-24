#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from AGV5.msg import target


def sometopictotwist(data):


	vel = Twist()
	vel.linear.x = data.linvel
	vel.angular.z = data.angvel
	velPub.publish(vel)

rospy.init_node('bruuuuuuuuuuuh', anonymous=True)
velPub = rospy.Publisher('velmsgs', Twist, queue_size=10)
rospy.Subscriber('/some/topic',target, sometopictotwist)


if __name__ == '__main__':

	rospy.spin()
