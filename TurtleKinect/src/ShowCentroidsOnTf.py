#!/usr/bin/env python  
import roslib
roslib.load_manifest('TurtleKinect')
import rospy

import tf
import geometry_msgs.msg

def transform_centroids(msg):
    br = tf.TransformBroadcaster()
    poses = msg.poses
    for p in range(len(poses)):
        br.sendTransform((poses[p].position.x, poses[p].position.y, poses[p].position.z),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "Centroid %d"%p,
                         msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('centroid_tf_broadcaster')
    rospy.Subscriber("/turtle_cluster/centroids",
                     geometry_msgs.msg.PoseArray,
                     transform_centroids)
    rospy.spin()