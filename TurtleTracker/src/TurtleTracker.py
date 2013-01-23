#!/usr/bin/env python  
import roslib
roslib.load_manifest('TurtleTracker')
import rospy
import tf
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('TurtleTracker')

    listener = tf.TransformListener()
    pub_vo = rospy.Publisher('/vo', Odometry)

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/kinect_top/camera_depth_frame', '/user_1', rospy.Time(0))
            msg = Odometry()
            msg.header.stamp = rospy.Time.now()           # time of gps measurement
            msg.header.frame_id = 'kinect'                # the tracked robot frame
            msg.pose.pose.position.x = trans[0]
            msg.pose.pose.position.y = trans[1]
            msg.pose.pose.position.z = trans[2]
            msg.pose.pose.orientation.x = 1
            msg.pose.pose.orientation.y = 0
            msg.pose.pose.orientation.z = 0
            msg.pose.pose.orientation.w = 0
            # Re-check the variances, I don't know if these values are good
            msg.pose.covariance = [1, 0, 0, 0, 0, 0,      # covariance on gps_x
                                   0, 1, 0, 0, 0, 0,      # covariance on gps_y
                                   0, 0, 1, 0, 0, 0,      # covariance on gps_z
                                   0, 0, 0, 99999, 0, 0,  # large covariance on rot x
                                   0, 0, 0, 0, 99999, 0,  # large covariance on rot y
                                   0, 0, 0, 0, 0, 99999]  # large covariance on rot z
            pub_vo.publish(msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
