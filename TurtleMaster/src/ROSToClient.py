#!/usr/bin/env python
import roslib; roslib.load_manifest('TurtleMaster')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import math
import socket
import sys
import json

class ROSToClient:
	def msgSend(self):
		self.package+=']\n'
		sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		try:
			#connect
			sock.connect((self.HOST, self.PORT))
			sock.sendall(self.package)
			received=sock.recv(1024)
		finally:
			sock.close()
		print "Sent: %s" %self.package.strip()
		#print "Received %s" %received
		self.package=None

	def prepSend(self,data):
		if(self.package==None):
			self.package='['+data
			self.last_time=rospy.Time.now().to_sec()
		elif len(self.package)+len(data)<1021:
			self.package+=','+data
			if rospy.Time.now().to_sec()-self.last_time > 0.01:
				self.msgSend()
		else:
			self.msgSend()
			self.prepSend(data)

	def callbackString(self,data):
		#rospy.loginfo(rospy.get_name()+"msg in %s",data.data)
		data_json=json.dumps(['S',data.data])
		self.prepSend(data_json)

	def callbackVector(self,data):
		#rospy.loginfo(rospy.get_name+"msg in %f %f %f"%data.x,data.y,data.z)
		data_json=json.dumps(['V',data.x,data.y,data.x])
		self.prepSend(data_json)

	def callbackTransform(self,data):
		#rospy.loginfo(rospy.get_name+"msg in %f %f %f"%data.x,data.y,data.z)
		tf.matrix3x3(data.rotation).getRPY(roll,pitch,yaw)
		data_json=json.dumps(['T',data.header.frame_id, [data.transform.translation.x, data.transform.translation.y ,data.transform.translation.z], [roll, pitch, yaw]])
		self.prepSend(data_json)

	def __init__(self,HOST, PORT):
		self.HOST,self.PORT = HOST, PORT
		rospy.init_node('turtle_serv', anonymous=True)
		#rospy.Subscriber('string_serv', String, self.callbackString)
		#rospy.Subscriber('vector_serv', Vector3, self.callbackVector)
		rospy.Subscriber('serv_transforms', TransformStamped, self.callbackTransform)
		self.package=None
		rospy.spin()


if __name__=='__main__':
	#first argument is the host
	server=ROSToClient(sys.argv[1],9003)
