#!/usr/bin/env python
"""Motion Prediction Node.
Uses position data from the camera to estimate the velocity and acceleration of the ball.
"""

import rospy

import numpy as np
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

class KalmanFilter:

    def __init__(self, err_measure, err_estimate, process_noise):
        self.err_measure = err_measure
        self.q = process_noise

        self.err_estimate = err_estimate
        self.last_estimate = 0

    def update(self, measure):
        gain = self.err_estimate / (self.err_estimate + self.err_measure)
        estimate = self.last_estimate + gain * (measure - self.last_estimate)
        self.err_estimate = (1 - gain) * self.err_estimate + abs(self.last_estimate - estimate) * self.q
        self.last_estimate = estimate
        return estimate

class KalmanFilter3D:

    def __init__(self, err_measure, err_estimate, process_noise):
        self.kx = KalmanFilter(err_measure, err_estimate, process_noise)
        self.ky = KalmanFilter(err_measure, err_estimate, process_noise)
        self.kz = KalmanFilter(err_measure, err_estimate, process_noise)

    def update(self, x, y, z):
        return [self.kx.update(x), self.ky.update(y), self.kz.update(z)]

def derivative(message, message_prev, field):
    dx = getattr(message.pose.position, field) - getattr(message_prev.pose.position, field)
    dt = message.header.stamp.nsecs - message_prev.header.stamp.nsecs
    dt = dt * 1e-9
    return dx / dt

def dbl_derivative(msg, msg_prev, msg_prev_prev, field):
    ddx = derivative(msg, msg_prev, field) - derivative(msg_prev, msg_prev_prev, field)
    dt = (msg.header.stamp.nsecs - msg_prev_prev.header.stamp.nsecs) / 2
    dt = dt * 1e-9
    return ddx/dt


if __name__ == '__main__':
    rospy.init_node('motion_predict')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('GoalPose', geometry_msgs.msg.PoseStamped, queue_size=10)
    rate = rospy.Rate(100.0)
    br = tf2_ros.TransformBroadcaster()

    initLoop = 1
    while initLoop:
        try:
            trans = tfBuffer.lookup_transform('ar_marker_5', 'ball_frame', rospy.Time())
            initLoop = 0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rate.sleep()

    lastx = trans.transform.translation.x
    lasty = trans.transform.translation.y
    dx = [0., 0., 0., 0.,0.]
    dy = [0., 0., 0., 0.,0.]
    y_ints = [0., 0., 0., 0., 0.]
    angles = [0., 0., 0., 0., 0.]

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ar_marker_5', 'ball_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()
            continue

        newdx = trans.transform.translation.x - lastx
        newdy = trans.transform.translation.y - lasty
        dx.pop(0)
        dy.pop(0)
        dx.append(newdx)
        print(dx, newdx)
        dy.append(newdy)

        # compute the angle of the velocity of the ball using the low_pass filtered dy and dx
        # angle = np.arctan2(np.mean(dx), np.mean(dy))
        angle = np.arctan2(newdx, newdy)
        angles.pop(0)
        angles.append(angle)
        newyint = (-trans.transform.translation.x) * np.tan(np.mean(angles)) + trans.transform.translation.y
        
        y_ints.pop(0)
        y_ints.append(newyint)        
        
        # low pass filter the y_intercepts
        y_intercept = np.mean(y_ints)

        # creating the message.
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = trans.header.stamp
        t.header.frame_id = "ar_marker_5"
        t.child_frame_id = "goal"
        t.transform.translation.x = 0
        t.transform.translation.y = y_intercept
        t.transform.translation.z = .1
        t.transform.rotation.w = 1
        br.sendTransform(t)
        rate.sleep()

