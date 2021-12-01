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
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2

from cv_bridge import CvBridge

def segment_pointcloud(points, segmented_image, cam_matrix, trans, rot):
    IDX2D = lambda i, j, dj: dj * i + j
    xyz = np.vstack((points['x'], points['y'], points['z']))
    pixel_coords = project_points(xyz, cam_matrix, trans, rot)

    image_h, image_w = segmented_image.shape[:2]

    in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                & (0 <= pixel_coords[1]) & (pixel_coords[1] < image_h))
    
    points = points[in_frame]
    pixel_coords = pixel_coords[:, in_frame]
    j, i = pixel_coords
    linearized_pixel_coords = IDX2D(i, j, segmented_image.shape[1])
    linearized_segmentation = segmented_image.reshape(-1)
    point_labels = linearized_segmentation[linearized_pixel_coords]
    return points[point_labels == 1]

def project_points(points, cam_matrix, trans, rot):
    points = np.dot(rot, points) + trans[:, None]
    homo_pixel_coords = np.dot(cam_matrix, points)
    pixel_coords = homo_pixel_coords[0:2, :] / homo_pixel_coords[2, :]
    pixel_coords = np.floor(pixel_coords).astype('int32')
    return pixel_coords

def isolate_tennis_ball(img):
    resized = cv2.resize(img, None, fx=0.25, fy=0.25)
    hsv = cv2.cvtColor(resized, cv2.COLOR_RGB2HSV)
    lower_thresh = np.array([25, 40, 40])
    upper_thresh = np.array([75, 255, 255])
    mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
    mask = cv2.erode(mask, np.ones((5, 5)))

    # cv2.imshow('window', mask)

    _, cnt, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    def score(cnt):
        return cv2.contourArea(cnt)
   
    if not len(cnt):
        return np.zeros_like(mask)
    best = max(cnt, key=score)
    M = cv2.moments(best)
    if M["m00"] == 0:
        return np.zeros_like(mask)
    x = M["m10"] / M["m00"]
    y = M["m01"] / M["m00"]

    masked = cv2.bitwise_and(resized, resized, mask=mask)

    cv2.circle(resized, (int(x), int(y)), 3, (255, 255, 255), thickness=0)
    mask2 = cv2.drawContours(np.zeros_like(mask), [best], -1, color=255, thickness=cv2.FILLED)
    # cv2.imshow('image', cv2.cvtColor(resized, cv2.COLOR_RGB2BGR))
    mask2 = cv2.resize(mask2, (img.shape[1], img.shape[0]))
    # cv2.imshow('image', mask2)
    # cv2.waitKey(1)
    return mask2/ 255

def get_camera_matrix(camera_info_msg):
    return np.array(camera_info_msg.K).reshape((3,3))

def isolate_object_of_interest(points, image, cam_matrix, trans, rot):
    segmented_image = isolate_tennis_ball(image)
    points = segment_pointcloud(points, segmented_image, cam_matrix, trans, rot)
    return points

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')

class PointcloudProcess:
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, points_sub_topic, 
                       image_sub_topic,
                       cam_info_topic,
                       points_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, points_msg, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/camera_color_optical_frame',
                                                       '/camera_depth_optical_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException):
                return
            points = isolate_object_of_interest(points, image, info, 
                np.array(trans), np.array(rot))
            points_msg = numpy_to_pc2_msg(points)
            self.points_pub.publish(points_msg)
            print("Published segmented pointcloud at timestamp:",
                   points_msg.header.stamp.secs)


class center_point:
    def __init__(self, points_sub_topic, center_pub_topic):
        self.messages = deque([], 5)
        self.num_steps = 0
        self.pose = PoseStamped()
        self.pose.header.frame_id = "camera_depth_optical_frame"

        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)

        self.center_pub = rospy.Publisher(center_pub_topic, PoseStamped, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, points_msg):
        try:
            # points = pc2.read_points(points_msg, field_names = ("x", "y", "z"), skip_nans=True)
            # center = points.mean(0)
            points = ros_numpy.numpify(points_msg)
            center = (0, 0, 0, 0)
            count = 0
            for point in points:
                count += 1
                center = map(lambda x, y: x+y, center, point)
            center = (center[0] / count, center[1] / count, center[2] / count)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft(center)

    def publish_once_from_queue(self):
        if self.messages:
            x, y, z = self.messages.pop()
            self.pose.header.stamp = rospy.Time.now()
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = z
            self.center_pub.publish(self.pose)
            print("Published center at timestamp:",
                   self.pose.header.stamp)

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    POINTS_PUB_TOPIC = 'segmented_points'
    CENTER_PUB_TOPIC = 'center_point'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    centerer = center_point(POINTS_PUB_TOPIC, CENTER_PUB_TOPIC)
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        centerer.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
