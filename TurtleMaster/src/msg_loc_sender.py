#!/usr/bin/env python
import roslib; roslib.load_manifest('TurtleMaster')
import rospy
from geometry_msgs.msg import Vector3
def talker():
	pub = rospy.Publisher('vector_serv', Vector3)
	rospy.init_node('vector_sender')
	while not rospy.is_shutdown():
		data=[1.1,2.2,3.3]
		pub.publish(Vector3(data[0],data[1],data[2]))
		#rospy.sleep(.005)

if __name__=='__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
