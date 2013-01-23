#!/usr/bin/env python  
import roslib
roslib.load_manifest('TurtleMaster')
import rospy
import math
import tf
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion

class transformListener:
	def __init__(self):	
		rospy.init_node('tf_listener')
		rospy.Subscriber("id_topic",String,self.callback)
		self.ids=['/base_link','/human']
		self.listener = tf.TransformListener()
		self.rate = rospy.Rate(10.0)
		self.pub=rospy.Publisher('serv_transforms',TransformStamped)
		self.i=0

		while not rospy.is_shutdown():
			try:
				for i in range(len(self.ids)):
					trans=self.listener.lookupTransform(self.ids[i],'/map', rospy.Time(0))
					myHeader=Header(self.i,rospy.Time(0),'/map')
					print "publishing transform for", self.ids[i], trans
					myTransform=Transform(Vector3(trans[0][0],trans[0][1],trans[0][2]),Quaternion(trans[1][0],trans[1][1],trans[1][2]))
					self.pub.publish(TransformStamped(myHeader,self.ids[i],Transform(trans[0],trans[1])))
					i+=1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			rate.sleep()

	def callback(self,data):
		self.ids=data.split(',')

if __name__ == '__main__':
	transformer=transformListener()
