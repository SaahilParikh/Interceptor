#!/usr/bin/env python
"""Motion Prediction Node.
Uses position data from the camera to estimate the velocity and acceleration of the ball.
"""

import rospy

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

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('ar_marker_5', 'ball_frame', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print(math.atan2(trans.transform.translation.y, trans.transform.translation.x))
        print(math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2))

        rate.sleep()
