#!/usr/bin/env python
"""
Path Planning Script for Lab 7
"""
import sys
from genpy import message
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

import tf2_ros

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb


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

    raw_input(bcolors.HEADER + bcolors.UNDERLINE + bcolors.BOLD + "[ACTION REQUIRED] CLICK ENTER TO START" + bcolors.ENDC)
    transform_base_to_gripper = get_table()

    def move():
        if(LOG):
            print(bcolors.OKGREEN + "[LOGGER] project_path: Moving Baxter's Arm" + bcolors.ENDC)
        
        try: 
            goal = PoseStamped()
            goal.header.frame_id = "goal"

            goal_transform = tfBuffer.lookup_transform("goal", "base", rospy.Time())
            goal.pose.position.x = 0
            goal.pose.position.y = 0
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
        except Exception as e:
            print(e)
            traceback.print_exc()
        if(LOG):
            print(bcolors.OKGREEN + "[LOGGER] project_path: Move Finished" + bcolors.ENDC)
        return

    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/String from the topic /chatter_talk.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        move()
        rate.sleep()
        #planner.stop_movement()

    exit()
    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    

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
