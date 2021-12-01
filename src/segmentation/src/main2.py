#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf

from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import numpy as np
import cv2

from cv_bridge import CvBridge

from image_segmentation import isolate_object_of_interest

def get_camera_matrix(camera_info_msg):
    # TODO: Return the camera intrinsic matrix as a 3x3 numpy array
    # by retreiving information from the CameraInfo ROS message.
    # Hint: numpy.reshape may be useful here.
    return np.array(camera_info_msg.K).reshape((3,3))

def numpy_to_image_msg(imageRGB):
    return ros_numpy.msgify(Image, imageRGB, stamp=rospy.Time.now(),
        frame_id='camera_image_isolated_frame')

class imageProcess:
    """
    Wraps the processing of an image from an input ros topic and publishing
    to another image topic.

    """
    def __init__(self, image_sub_topic,
                       cam_info_topic,
                       image_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.image_pub = rospy.Publisher('ball_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        print('got to before the callback')
        ts.registerCallback(self.callback)

    def callback(self, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
        except Exception as e:
            print('encountered an error')
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((rgb_image, intrinsic_matrix))

    def publish_once_from_queue(self):
        if self.messages:
            print('starting to publish?')
            image, info = self.messages.pop()
            isolated_ball_pixels = isolate_object_of_interest(image, info)
            image_msg = numpy_to_image_msg(isolated_ball_pixels)
            self.image_pub.publish(image_msg)
            print("Published isolated ball pixel image at timestamp:",
                   image_msg.header.stamp.secs)

def main():
    CAM_INFO_TOPIC = '/usb_cam/camera_info'
    RGB_IMAGE_TOPIC = '/usb_cam/image_raw'
    IMAGE_PUB_TOPIC = '/usb_cam/ball_isolated'

    rospy.init_node('realsense_listener')
    print('initialized node')
    process = imageProcess(RGB_IMAGE_TOPIC, CAM_INFO_TOPIC, IMAGE_PUB_TOPIC)
    r = rospy.Rate(1000)

    print('initialized processor')
    while not rospy.is_shutdown():

        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
