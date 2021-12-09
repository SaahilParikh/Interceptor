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
from filterpy.common import Q_discrete_white_noise
import time
import numpy as np

f = KalmanFilter(dim_x=4, dim_z=2)

# https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node

# measurement matrix: how to compute the measured state (position) from the tracked state (position + velocity)
f.H = np.array([[1, 0, 0, 0],
                [0, 0, 1, 0]])

f.P *= 1000 # covariance matrix
f.R = 5 # measurement noise



if __name__ == '__main__':
    rospy.init_node('motion_predict')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('GoalPose', geometry_msgs.msg.PoseStamped, queue_size=10)
    rate = rospy.Rate(100.0)
    br = tf2_ros.TransformBroadcaster()


    xs = [0, 0, 0]
    ys = [0, 0, 0]
    goals = list(np.zeros(20))
 
    counter = 0
    last_time = time.time()
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ar_marker_6', 'ball_frame', rospy.Time())
            ctime = time.time()
            # print(trans.header)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

        # https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node
        dt = ctime - last_time
        last_time = ctime
        f.F = np.array([[1, dt,  0,  0],
                      [0,  1,  0,  0],
                      [0,  0,  1, dt],
                      [0,  0,  0,  1]])

        # f.B = np.array([dt**2/2, dt, dt**2/2, dt])
        # f.Q = Q_discrete_white_noise(dim=4, dt=dt, var=.05)

        nx, ny = trans.transform.translation.x, trans.transform.translation.y
        xs.pop(0)
        ys.pop(0)
        xs.append(nx)
        ys.append(ny)
        z = np.array([np.mean(xs), np.mean(ys)])
        f.predict()
        f.update(z)

        (x, vx, y, vy) = f.x

        v_angle = math.atan2(vy, vx)
        velocity = (math.sqrt(vx ** 2 + vy ** 2))
        # if velocity > 0.2:
        #     print('angle', angle/math.pi*180, 'velocity', velocity)

        # print(f.x, v_angle, velocity)      
        counter += 1
        if (velocity < .01):
            y_int = -x * math.tan(v_angle) + y
        else:
            y_int = y


        if (y_int - np.mean(goals) < .2):
            y_intercept = y_int
        goals.pop(0)
        goals.append(y_int)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = trans.header.stamp
        t.header.frame_id = "ar_marker_6"
        t.child_frame_id = "goal"
        t.transform.translation.x = 0
        t.transform.translation.y = y_intercept
        t.transform.rotation.w = 1
        br.sendTransform(t)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "ar_marker_6"
        p.header.stamp = trans.header.stamp
        p.pose.position.x = 0
        p.pose.position.y = y_intercept
        pub.publish(p)
        rate.sleep()
