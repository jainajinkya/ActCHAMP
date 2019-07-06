#!/usr/bin/env python

import sys, select

import rospy
from sensor_msgs.msg import JointState

state = None

def callback(msg):
    global state
    state = msg.position[:7]


if __name__ == "__main__":
    global state 

    rospy.init_node('saved_trajectory_playback', anonymous=True)
    rospy.Subscriber("/vector/right_arm/joint_states", JointState, callback)
    
    while not rospy.is_shutdown():
        text = raw_input("\nPress Enter To Capture joint state! ")

        if text == "":
          print "Robot State: ", state
