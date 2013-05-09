#!/usr/bin/env python
import roslib
roslib.load_manifest('TurtleTracker')
import rospy
import SocketServer
import sys
import threading
import socket
import time
import tf

tf_listener = None


class CentroidClient(SocketServer.BaseRequestHandler):

    def handle(self):
        global master
        while not rospy.is_shutdown():
            try:
                # self.request is the TCP socket connected to the client
                self.data = self.request.recv(4096).strip()
                try:
                    for i in range(20):
                        try:
                            (trans, rot) = tf_listener.lookupTransform(
                                '/camera_depth_frame', '/user_%d' % i, rospy.Time(0))
                            msg = '{" x":%0.4f," y":%0.4f," z":%0.4f, "rx":%0.4f, "ry":%0.4f, "rz":%0.4f }\n' % (
                                trans[0], trans[1], trans[2], rot[0], rot[1], rot[2])
                            self.request.sendall(msg)
                            break
                        except:
                            pass
                except:  # data errors
                    # send to log and other side
                    err = [sys.exc_info()[0].__name__, str(sys.exc_info()[1])]
                    rospy.loginfo(err)
                    self.request.sendall(str(err)+'\n')
            except:  # socket errors
                rospy.loginfo([sys.exc_info()[
                              0].__name__, str(sys.exc_info()[1])])
                break


# threaded server handles connections in async fashion
class TurtleTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    allow_reuse_address = True

if __name__ == "__main__":
    rospy.init_node('turtle_centroid_server')  # initialize ros node
    tf_listener = tf.TransformListener()

    HOST, PORT = sys.argv[1], sys.argv[2]
    server = TurtleTCPServer((HOST, PORT), CentroidClient)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # serve in separate thread
    server_thread = threading.Thread(target=server.serve_forever)
    # exit the server thread when the main thread terminates, unbinds port
    server_thread.daemon = True
    server_thread.start()

    rospy.spin()
    server.shutdown()
