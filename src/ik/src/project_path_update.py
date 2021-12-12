#!/usr/bin/env python
"""
Path Planning Script for Lab 7
"""
import sys
from genpy import message
ROBOT = "sawyer"

import tf2_ros

from intera_interface import Limb

from os import system, name
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from path_planner import PathPlanner
try:
    from controller import Controller
except ImportError:
    pass

LOG = False

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def main():

    if ROBOT == "sawyer":
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    else:
        Kp = 0.45 * np.array([0.8, 2.5, 1.7, 2.2, 2.4, 3, 4])
        Kd = 0.015 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    poopoo = Controller(Kp, Ki, Kd, Kw, Limb('right') if ROBOT == "baxter" else Limb() )

    planner = PathPlanner("right_arm")


    ## ADD OBSTACLE ##
    q = [0.0, 0.00, -0.24]
    w = [0.00, 0.00, 0.00, 1.00]
    size = [3.0, 3.0, 0.1]
    
    table = PoseStamped()
    table.header.frame_id = "base"

    #x, y, and z position
    table.pose.position.x = q[0]
    table.pose.position.y = q[1]
    table.pose.position.z = q[2]

    table.pose.orientation.x = w[0]
    table.pose.orientation.y = w[1]
    table.pose.orientation.z = w[2]
    table.pose.orientation.w = w[3]

    planner.add_box_obstacle(np.array(size), "table", table)

    q_s = [-0.5, 0.0, 0.00]
    w_s = [0.00, -1.00, 0.00, 0.00]
    size_s = [.3, 3.0, 3.0]
    
    wall_side = PoseStamped()
    wall_side.header.frame_id = "base"

    #x, y, and z position
    wall_side.pose.position.x = q_s[0]
    wall_side.pose.position.y = q_s[1]
    wall_side.pose.position.z = q_s[2]

    wall_side.pose.orientation.x = w_s[0]
    wall_side.pose.orientation.y = w_s[1]
    wall_side.pose.orientation.z = w_s[2]
    wall_side.pose.orientation.w = w_s[3]

    planner.add_box_obstacle(np.array(size_s), "side_wall", wall_side)

    q_b = [0.0, 0.7, 0.00]
    w_b = [0.00, -1.00, 0.00, 0.00]
    size_b = [3.0, .3, 3.0]
    
    wall_back = PoseStamped()
    wall_back.header.frame_id = "base"

    #x, y, and z position
    wall_back.pose.position.x = q_b[0]
    wall_back.pose.position.y = q_b[1]
    wall_back.pose.position.z = q_b[2]

    wall_back.pose.orientation.x = w_b[0]
    wall_back.pose.orientation.y = w_b[1]
    wall_back.pose.orientation.z = w_b[2]
    wall_back.pose.orientation.w = w_b[3]

    planner.add_box_obstacle(np.array(size_b), "back_wall", wall_back)

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper"
    orien_const.header.frame_id = "base"
    orien_const.orientation.y = -1.0
    orien_const.absolute_x_axis_tolerance = 0.1
    orien_const.absolute_y_axis_tolerance = 0.1
    orien_const.absolute_z_axis_tolerance = 0.1
    orien_const.weight = 1.0


    raw_input(bcolors.HEADER + bcolors.UNDERLINE + bcolors.BOLD + "[ACTION REQUIRED] CLICK ENTER TO START" + bcolors.ENDC)
    transform_base_to_gripper = get_table()

    y_list = [0 for i in range(5)]

    def move():
        if(LOG):
            print(bcolors.OKGREEN + "[LOGGER] project_path: Moving Baxter's Arm" + bcolors.ENDC)
        
        try: 
            goal = PoseStamped()
            goal.header.frame_id = "ar_marker_6"

            goal.pose.position.x = x_place
            goal.pose.position.y = y_list[-1]
            goal.pose.position.z = 0.05

            #Orientation as a quaternion
            goal.pose.orientation.x = 0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 0

            if (LOG):
                print(bcolors.OKBLUE + "[LOGGER] project_path: Goal Pose set to:\n" + str(goal) + bcolors.ENDC)

            # Might have to edit this . . . 
            plan = planner.plan_to_pose(goal, [])

            #raw_input("Press <Enter> to move the right arm to position: {p} and orientation: {q}")
            if not poopoo.execute_path(plan, log=False):
                raise Exception("Execution failed")
            else:
                return y_list[-1]
        except Exception as e:
            print(e)
            traceback.print_exc()
        
        if(LOG):
            print(bcolors.OKGREEN + "[LOGGER] project_path: Move Finished" + bcolors.ENDC)
        
        return None

    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/String from the topic /chatter_talk.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    
    rate = rospy.Rate(50)
    goal_transform = get_goal()
    x_place = goal_transform.transform.translation.x
    last_y = 0
    i = 0
    while not rospy.is_shutdown():
        goal_transform = get_goal()
        y_list.append(-goal_transform.transform.translation.y)
        y_list.pop(0)
        if (np.std(y_list) < 0.1 and abs(last_y - np.mean(y_list)) > 0.08):
            print("Move")
            new_y = move()
            if new_y is not None:
                last_y = new_y
        rate.sleep()
        #planner.stop_movement()

    exit()
    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C


def get_goal():
    while True:
        try:
            trans = tfBuffer.lookup_transform("goal", "ar_marker_6", rospy.Time())
            return trans

        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as e:
            print(e)
        rospy.sleep(1)

def get_table():
    while True:
        try:
            trans = tfBuffer.lookup_transform("base", "right_gripper_tip", rospy.Time())
            print(bcolors.OKCYAN + "[LOGGER] project_path: Transformation from base to right gripper found" + bcolors.ENDC)
            return trans

        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as e:
            print(e)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    main()


#rostopic pub robot/update_position geometry_msgs/PoseStamped '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: "base"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: -1.0, z: 0.0, w: 0.0}}}'
