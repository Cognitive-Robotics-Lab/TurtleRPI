#!/usr/bin/python
import socket
import sys
import json
#import roslib
#import rospy

def send(x,y,z,HOST, PORT):
	vector=[x,y,z]
	data = json.dumps(vector)
	# Create a socket (SOCK_STREAM means a TCP socket)
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	try:
		# Connect to server and send data
		sock.connect((HOST, PORT))
		sock.sendall(data + '\n')
		# Receive data from the server and shut down
		received=sock.recv(1024)
	finally:
		sock.close()
	print "Vector:     ", vector
	print "JSON:     ", data
	print "Received: ", received

if __name__=='__main__':
	HOST, PORT=str(sys.argv[1]),8003
	while True: #not rospy.is_shutdown():
		x=input('x: ')
		y=input('y: ')
		z=input('z: ')
		send(x,y,z,HOST, PORT)
