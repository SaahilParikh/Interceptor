#!/usr/bin/env python
"""Motion Prediction Node.
Uses position data from the camera to estimate the velocity and acceleration of the ball.
"""

import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
from filterpy.kalman import KalmanFilter
import time

f = KalmanFilter(dim_x=4, dim_z=2)

# https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node

# measurement matrix: how to compute the measured state (position) from the tracked state (position + velocity)
f.H = np.array([[1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 0]])

f.P *= 1000 # covariance matrix
f.R *= 5 # measurement noise
import numpy as np


if __name__ == '__main__':
    rospy.init_node('motion_predict')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('GoalPose', geometry_msgs.msg.PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    br = tf2_ros.TransformBroadcaster()

    last_time = time.time()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ar_marker_5', 'ball_frame', rospy.Time())
            ctime = time.time()
            print(trans.header)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

        # https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node
        dt = ctime - last_time
        last_time = ctime
        F = np.array([[1, dt,  0,  0],
                      [0,  1,  0,  0],
                      [0,  0,  1, dt],
                      [0,  0,  0,  1]])

        z = np.array([trans.transform.translation.x, trans.transform.translation.y])
        f.predict()
        f.update(z)

        print(f.x)

        (x, vx, y, vy) = f.x

        v_angle = math.atan2(vy, vx)
        velocity = (math.sqrt(vx ** 2 + vy.y ** 2))
        # if velocity > 0.2:
        #     print('angle', angle/math.pi*180, 'velocity', velocity)

        y_intercept = x * math.tan(v_angle)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = trans.header.stamp
        t.header.frame_id = "ar_marker_5"
        t.child_frame_id = "goal"
        t.transform.translation.x = 0
        t.transform.translation.y = y_intercept
        t.transform.rotation.w = 1
        br.sendTransform(t)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "ar_marker_5"
        p.header.stamp = trans.header.stamp
        p.pose.position.x = 0
        p.pose.position.y = y_intercept
        pub.publish(p)
        rate.sleep()
