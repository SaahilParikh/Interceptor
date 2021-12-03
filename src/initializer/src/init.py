#!/usr/bin/env python
""" this node publishes the frame of the ball with a parent of the camera_depth_optical_frame"""

from __future__ import print_function

import rospy
import tf2_ros
import tf2_geometry_msgs

import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    loop = True
    rate = rospy.Rate(10.0)
    while loop:
        try: 
            t = tf_buffer.lookup_transform("ar_marker2_5", 'base', rospy.Time())
            loop = False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.frame_id = "ar_marker_5"
    static_transformStamped.child_frame_id = "base"
    static_transformStamped.transform.translation = t.transform.translation
    static_transformStamped.transform.rotation = t.transform.rotation

    while not rospy.is_shutdown():
        static_transformStamped.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(static_transformStamped)
