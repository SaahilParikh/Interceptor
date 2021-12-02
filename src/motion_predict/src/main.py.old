#!/usr/bin/env python
"""Motion Prediction Node.
Uses position data from the camera to estimate the velocity and acceleration of the ball.

Adapted from Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena
"""

from __future__ import print_function
from collections import deque

import rospy
import math

from geometry_msgs.msg import PoseStamped, Twist
#from motion_predict.msg import Motion
#import matplotlib.pyplot as plt


#plt.axis([0, 1000, 0, 1])
#plt.figure()

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

i= 0

class MotionPredictProcess:

    def __init__(self, points_sub_topic, motion_pub_topic):

        points_sub = rospy.Subscriber(points_sub_topic, PoseStamped, self.callback)

        self.motion_pub= rospy.Publisher(motion_pub_topic, Twist, queue_size=10)
        
        #ts.registerCallback(self.callback)

        self.pos_queue = deque(maxlen=5)
        self.velocity_kf = KalmanFilter3D(20, 2, 0.01)
        self.acceleration_kf = KalmanFilter3D(20, 2, 0.001)

    def callback(self, message):
        global i
        print(message)
        self.pos_queue.append(message)
        if len(self.pos_queue) > 3:
            velocity_x = derivative(self.pos_queue[-1], self.pos_queue[-2], 'x')
            velocity_y = derivative(self.pos_queue[-1], self.pos_queue[-2], 'y')
            velocity_z = derivative(self.pos_queue[-1], self.pos_queue[-2], 'z')

            print((velocity_x**2 + velocity_y**2 + velocity_z**2)**0.5)
            m = 5
            if abs(velocity_x) > m or abs(velocity_y) > m or abs(velocity_z) > m:
                return
            
            #plt.scatter(i, velocity_z)
            #plt.pause(0.001)
            #i = i + 1

            print('instantaneous', [velocity_x, velocity_y, velocity_z])
            velocity = self.velocity_kf.update(velocity_x, velocity_y, velocity_z)
            
            acceleration_x = dbl_derivative(self.pos_queue[-1], self.pos_queue[-2], self.pos_queue[-3], 'x')
            acceleration_y = dbl_derivative(self.pos_queue[-1], self.pos_queue[-2], self.pos_queue[-3], 'y')
            acceleration_z = dbl_derivative(self.pos_queue[-1], self.pos_queue[-2], self.pos_queue[-3], 'z')
            acceleration = self.acceleration_kf.update(acceleration_x, acceleration_y, acceleration_z)
            
            print('velocity:', velocity)
            print('acceleration', acceleration)
            
            velocity_angle = math.atan2(velocity_y, velocity_x)
            y_intercept = self.pos_queue[-1].pose.position.x * math.tan(velocity_angle)
            print(y_intercept)
 		
            t = Twist()
            t.linear.x = velocity[0]
            t.linear.y = velocity[1]
            t.linear.z = velocity[2]

            t.angular.x = acceleration[0]
            t.angular.y = acceleration[1]
            t.angular.z = acceleration[2]
            
            self.motion_pub.publish(t)

def main():
    POINTS_TOPIC = '/center_point'
    MOTION_PUB_TOPIC = 'motion'

    rospy.init_node('motion_predict')
    process = MotionPredictProcess(POINTS_TOPIC, MOTION_PUB_TOPIC)
    #plt.show(block=True)
    rospy.spin()

if __name__ == '__main__':
    main()
