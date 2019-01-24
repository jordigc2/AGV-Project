#!/usr/bin/env python
# license removed for brevity
import rospy
from AGV5.msg import target


def talker():
	pub = rospy.Publisher('/some/topic', target, queue_size=1000)

	rospy.init_node('target_dude', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		target_vels = target()
		target_vels.linvel = 0.2
		target_vels.angvel = 0.0
		pub.publish(target_vels)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
