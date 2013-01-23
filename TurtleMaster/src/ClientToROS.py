#!/usr/bin/env python
import roslib; roslib.load_manifest('TurtleMaster')
import rospy
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
from TurtleMsgSrv.srv import Goto
import json
import SocketServer
import sys
import threading
import socket
import time

class ServToROS:
	def __init__(self):
		#self.pub=rospy.Publisher('from_serv',Vector3Stamped)
		#self.pub=rospy.Publisher('from_serv',Vector3)
		#goto command server
		self.goto_serv=rospy.ServiceProxy('goto',Goto)
		self.i=0

	def process_message(self,data):
		#rospy.loginfo(data)
		data = ['/base_link']+data

		#validate all the things here, drew's code:
		#http://docs.python.org/2/library/exceptions.html
		if len(data)!=4:
			raise TypeError("goto requires 4 parameters")

		#self.pub.publish(Vector3(float(unpacked[0]),float(unpacked[1]),float(unpacked[2])))
		#self.pub.publish(Vector3Stamped(Header(self.i,rospy.Time.now(),'global'), Vector3(float(unpacked[0]),float(unpacked[1]),float(unpacked[2]))))
		self.goto_serv(*data)
		self.i+=1

class ClientToServ(SocketServer.BaseRequestHandler):
	def handle(self):
		global master
		while not rospy.is_shutdown():
			try:
				#self.request is the TCP socket connected to the client
				self.data=self.request.recv(4096).strip()
				print "len:", len(self.data)
				print "the data is:",self.data
				try:
					unpacked=json.loads(self.data)
					master.process_message(unpacked)
					self.request.sendall("Processing\n")
				except: #data errors
					#send to log and other side
					err = [sys.exc_info()[0].__name__,str(sys.exc_info()[1])]
					rospy.loginfo(err)
					self.request.sendall(str(err)+'\n')
			except: #socket errors
				rospy.loginfo([sys.exc_info()[0].__name__,str(sys.exc_info()[1])])
				break


#threaded server handles connections in async fashion
class TurtleTCPServer(SocketServer.ThreadingMixIn,SocketServer.TCPServer):
	allow_reuse_address = True
	
if __name__=="__main__":
	rospy.init_node('turtle_rec') #initialize ros node
	global master
	master = ServToROS()

	HOST, PORT=sys.argv[1],8003
	server = TurtleTCPServer((HOST,PORT),ClientToServ)
	server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	#serve in separate thread 
	server_thread = threading.Thread(target=server.serve_forever)
	#exit the server thread when the main thread terminates, unbinds port
	server_thread.daemon = True
	server_thread.start()

	rospy.spin()
	server.shutdown()
