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
import csv

f = KalmanFilter(dim_x=4, dim_z=2)

# https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node

# measurement matrix: how to compute the measured state (position) from the tracked state (position + velocity)
f.H = np.array([[1, 0, 0, 0],
                [0, 0, 1, 0]])

f.P *= 0.1**2 # covariance matrix
f.R = np.array([
    [0.001**2, 0],
    [0, 0.001**2],
]) # measurement noise


if __name__ == '__main__':

    csv_file = open('out.txt', 'wb')
    writer = csv.writer(csv_file)
    writer.writerow(['x', 'y', 'kx', 'ky', 'kvx', 'kvy'])

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

    y_intercept = 0

    positions = None

    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ar_marker_6', 'ball_frame', rospy.Time())
            ctime = time.time()
            # print(trans.header)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

        dt = (trans.header.stamp - last_time).to_sec()
        last_time = trans.header.stamp
        if dt == 0:
            continue

        # https://dsp.stackexchange.com/questions/26115/kalman-filter-to-estimate-3d-position-of-a-node
        # dt = ctime - last_time
        # last_time = ctime
        f.F = np.array([[1, dt,  0,  0],
                      [0,  1,  0,  0],
                      [0,  0,  1, dt],
                      [0,  0,  0,  1]])
        f.Q = Q_discrete_white_noise(dim=2, dt=dt, var=.0005, block_size=2)

        # f.B = np.array([dt**2/2, dt, dt**2/2, dt])
        nx, ny = trans.transform.translation.x, trans.transform.translation.y
        z = np.array([nx, ny])

        # PRINT STD OF DATA
        # if positions is None:
        #     positions = np.array([z])
        # else:
        #     positions = np.vstack([positions, z])
        # print(np.std(positions, axis=0))

        f.predict()
        f.update(z)

        (x, vx, y, vy) = f.x

        v_angle = math.atan2(vy, vx)
        velocity = (math.sqrt(vx ** 2 + vy ** 2))
        # if velocity > 0.2:
        #     print('angle', angle/math.pi*180, 'velocity', velocity)

        # print(f.x, v_angle, velocity)      
        if (velocity > .01):
            y_int = -x * math.tan(v_angle) + y
        else:
            y_int = y

        print(velocity)
        writer.writerow([nx, ny, x[0], y[0], vx[0], vy[0]])

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = trans.header.stamp
        t.header.frame_id = "ar_marker_6"
        t.child_frame_id = "goal"
        t.transform.translation.x = 0
        t.transform.translation.y = y_int
        t.transform.rotation.w = 1
        br.sendTransform(t)

        t.child_frame_id = "est_ball"
        t.transform.translation.x = x
        t.transform.translation.y = y
        br.sendTransform(t)


        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "ar_marker_6"
        p.header.stamp = trans.header.stamp
        p.pose.position.x = 0
        p.pose.position.y = y_int
        pub.publish(p)
        rate.sleep()
