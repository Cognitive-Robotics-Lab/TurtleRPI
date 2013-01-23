#!/usr/bin/env python
import roslib; roslib.load_manifest('TurtleMaster')
import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped

def callback(data):
	rospy.loginfo(rospy.get_name()+"vector in %f %f %f",data.vector.x,data.vector.y,data.vector.z)
	#rospy.sleep(1.0)

def listener():
	rospy.init_node('msg_receiver', anonymous=True)
	rospy.Subscriber('from_serv', Vector3Stamped, callback)
	#rospy.Subscriber('from_serv', Vector3, callback)
	rospy.spin()

if __name__=='__main__':
	listener()
