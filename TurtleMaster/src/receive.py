#!/usr/bin/python
import SocketServer
import sys
import json

class TurtleServer(SocketServer.BaseRequestHandler):

	def handle(self):
		# self.request is the TCP socket connected to the client
		self.data = self.request.recv(1024).strip()
		self.unpacked=json.loads(self.data)
		#print self.data
		print 'received'
		print self.unpacked
		# just send back the same data, but upper-cased
		self.request.sendall(self.data)



if __name__ == "__main__":
	HOST, PORT = sys.argv[1], 9003

	# Create the server, binding to localhost on port 9999
	server = SocketServer.TCPServer((HOST, PORT), TurtleServer)

	# Activate the server; this will keep running until you
	# interrupt the program with Ctrl-C
	server.serve_forever()
