#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

# Define the method which contains the node's main functionality
def ball_location_pub():

    # Create an instance of the rospy.Publisher object
    pub = rospy.Publisher('ball_location', PoseStamped, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(100) # 10hz

    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.header.stamp.secs = 0
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        noisex = np.random.normal(0,0.001)
        noisey = np.random.normal(0,0.001)

        pose.pose.position.x = pose.pose.position.x + .01 + noisex
        pose.pose.position.y = pose.pose.position.y + .01 + noisey
        pose.pose.position.z = pose.pose.position.z 
        pose.header.stamp = rospy.Time.now()
        pub_pose = pose
        
        # Publish our string to the 'chatter_talk' topic
        pub.publish(pub_pose)
        print(rospy.get_name() + ": I sent \"%s\"" % pub_pose)
        
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('ball_pose', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        ball_location_pub()
    except rospy.ROSInterruptException: pass
