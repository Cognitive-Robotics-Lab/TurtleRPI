#!/usr/bin/env python
import roslib; roslib.load_manifest('TurtleMaster')
import rospy
from geometry_msgs.msg import Vector3Stamped, Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from TurtleMsgSrv.srv import Goto
import json
import SocketServer
import sys
import threading
import socket
import time



msg = """
Control Your TurtleBot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

pub = 0

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

class TurtleReceiver(SocketServer.BaseRequestHandler):
	def handle(self):
		global pub
		global msg
		moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
		}

		speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		}

		speed = .3
		turn = 1	
		x = 0
		th = 0
		status = 0
		pr = msg
		self.request.sendall(str(pr)+'\n')
		while not rospy.is_shutdown():
			try:
				#self.request is the TCP socket connected to the client
				self.data=self.request.recv(4096).strip()
				#print "len:", len(self.data)
				#print "the data is:",self.data
				print 'Key: ',self.data
				key = self.data
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					th = moveBindings[key][1]
				elif key in speedBindings.keys():
					speed = speed * speedBindings[key][0]
					turn = turn * speedBindings[key][1]

					pr = vels(speed,turn)
					rospy.loginfo(pr)
					self.request.sendall(str(pr)+'\n')
					if (status == 14):
						pr = msg
						rospy.loginfo(pr)
						self.request.sendall(str(pr)+'\n')
						status = (status + 1) % 15
				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)

			except: #data errors
				#send to log and other side
				err = [sys.exc_info()[0].__name__,str(sys.exc_info()[1])]
				rospy.loginfo(err)
				self.request.sendall(str(err)+'\n')
				twist = Twist()
				twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
				pub.publish(twist)
				break


#threaded server handles connections in async fashion
class TurtleTCPServer(SocketServer.ThreadingMixIn,SocketServer.TCPServer):
	allow_reuse_address = True
	
if __name__=="__main__":
	rospy.init_node('turtlebot_socket_teleop') #initialize ros node
	pub = rospy.Publisher('cmd_vel', Twist)



	HOST, PORT=sys.argv[1],8003
	server = TurtleTCPServer((HOST,PORT),TurtleReceiver)
	server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	#serve in separate thread 
	server_thread = threading.Thread(target=server.serve_forever)
	#exit the server thread when the main thread terminates, unbinds port
	server_thread.daemon = True
	server_thread.start()

	rospy.spin()
	server.shutdown()
