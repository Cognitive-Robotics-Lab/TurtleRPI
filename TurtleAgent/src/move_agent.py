#! /usr/bin/env python
import roslib
roslib.load_manifest('TurtleAgent')
import rospy
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from TurtleMsgSrv.srv import *
import tf
import actionlib
import math


class MoveAgent(object):

    def __init__(self):
        rospy.init_node('move_agent')
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self.goto_server()
        self.tf_lis = tf.TransformListener()
        self.ids = ['/base_link', '/human']
        # self.cancel_move_server()
        rospy.spin()

    def goto_server(self):
        self.goto = rospy.Service('goto', Goto, self.goto_server_handle)

    def goto_server_handle(self, request):
        print request
        if(request.move_agent_id not in self.ids):
            print("Id not in id list")
            return GotoResponse(False)

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = '/map'

        agent_pose = self.tf_lis.lookupTransform(
            '/map', request.move_agent_id, rospy.Time(0))
        print agent_pose
        agent_rot = tf.transformations.euler_from_quaternion(agent_pose[1])
        print agent_rot
        agent_rot = agent_rot[2]
        print (0, 0, request.rt)
        print (0, 0, request.rt+agent_rot)
        rot = tf.transformations.quaternion_from_euler(
            0, 0, request.rt+agent_rot)

        dx = math.cos(agent_rot)*request.dx - math.sin(agent_rot)*request.dy
        dy = math.sin(agent_rot)*request.dx + math.cos(agent_rot)*request.dy

        goal.target_pose.pose.orientation = Quaternion(*rot)
        goal.target_pose.pose.position.x = agent_pose[0][0] + dx
        goal.target_pose.pose.position.y = agent_pose[0][1] + dy
        print goal.target_pose.pose
        self.move_base_client.send_goal(goal)
        if(self.move_base_client.get_result() == actionlib.TerminalState.SUCCEEDED):
            print("Hooray, the base moved 1 meter forward")
            return GotoResponse(True)
        else:
            print("The base failed to move forward 1 meter for some reason")
            return GotoResponse(False)

if __name__ == "__main__":
    MoveAgent()
