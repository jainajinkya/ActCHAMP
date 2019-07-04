#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
from scipy.optimize import *
from copy import deepcopy


def array2PoseStamped(pos_array, frame=None):
    t = geometry_msgs.msg.PoseStamped()
    t.header.stamp = rospy.Time.now()

    if(frame is not None):
        t.header.frame_id = frame

    t.pose.position.x = pos_array[0]
    t.pose.position.y = pos_array[1]
    t.pose.position.z = pos_array[2]
    quat = tf.transformations.quaternion_from_euler(pos_array[3], pos_array[4], pos_array[5])
    t.pose.orientation.x = quat[0]
    t.pose.orientation.y = quat[1]
    t.pose.orientation.z = quat[2]
    t.pose.orientation.w = quat[3]
    return t

def array2Pose(pos_array):
    t = geometry_msgs.msg.Pose()
    t.position.x = pos_array[0]
    t.position.y = pos_array[1]
    t.position.z = pos_array[2]
    quat = tf.transformations.quaternion_from_euler(pos_array[3], pos_array[4], pos_array[5])
    t.orientation.x = quat[0]
    t.orientation.y = quat[1]
    t.orientation.z = quat[2]
    t.orientation.w = quat[3]
    return t

def poseStamped2Array(posStamp, quat2Euler=True):
    arr = []
    arr.append(posStamp.pose.position.x)
    arr.append(posStamp.pose.position.y)
    arr.append(posStamp.pose.position.z)

    if quat2Euler:
        quat = [posStamp.pose.orientation.x, posStamp.pose.orientation.y, posStamp.pose.orientation.z, posStamp.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        arr.append(euler[0])
        arr[4].append(euler[1])
        arr[5].append(euler[2])
    
    else:
        arr.append(posStamp.pose.orientation.x)
        arr.append(posStamp.pose.orientation.y)
        arr.append(posStamp.pose.orientation.z)
        arr.append(posStamp.pose.orientation.w)

    return np.array(arr)


def pose2Array(pos, quat2Euler=True):
    arr = []
    arr.append(pos.position.x)
    arr.append(pos.position.y)
    arr.append(pos.position.z)

    if quat2Euler:
        quat = [pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        arr.append(euler[0])
        arr.append(euler[1])
        arr.append(euler[2])
    else:
        arr.append(pos.orientation.x)
        arr.append(pos.orientation.y)
        arr.append(pos.orientation.z)
        arr.append(pos.orientation.w)
    
    return np.array(arr)

def pose2PoseStamped(pos, frame):
    return array2PoseStamped(pose2Array(pos), frame)


def transform2Pose(trans):
    return array2Pose(transform2Array(trans))

def transform2PoseStamped(trans, parent_frame=None, timeStamp=None):
    pos = geometry_msgs.msg.PoseStamped()

    if parent_frame != None:
        pos.header.frame_id = parent_frame

    if timeStamp != None:
        pos.header.stamp = timeStamp
    else:
        pos.header.stamp =rospy.Time.now()

    pos.pose.position.x = trans.transform.translation.x
    pos.pose.position.y = trans.transform.translation.y
    pos.pose.position.z = trans.transform.translation.z
    pos.pose.orientation.x = trans.transform.rotation.x
    pos.pose.orientation.y = trans.transform.rotation.y
    pos.pose.orientation.z = trans.transform.rotation.z
    pos.pose.orientation.w = trans.transform.rotation.w
    return pos

def transformStamped2PoseStamped(trans, timeStamp=None):
    pos = geometry_msgs.msg.PoseStamped()

    if timeStamp != None:
        pos.header.stamp = timeStamp
    else:
        pos.header.stamp =rospy.Time.now()

    pos.header.frame_id = trans.header.frame_id
    pos.pose.position.x = trans.transform.translation.x
    pos.pose.position.y = trans.transform.translation.y
    pos.pose.position.z = trans.transform.translation.z
    pos.pose.orientation.x = trans.transform.rotation.x
    pos.pose.orientation.y = trans.transform.rotation.y
    pos.pose.orientation.z = trans.transform.rotation.z
    pos.pose.orientation.w = trans.transform.rotation.w
    return pos


def transform2Array(trans):
    arr = np.zeros(6)
    arr[0] = trans.transform.translation.x
    arr[1] = trans.transform.translation.y
    arr[2] = trans.transform.translation.z
    quat = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
    euler = tf.transformations.euler_from_quaternion(quat)
    arr[3] = euler[0]
    arr[4] = euler[1]
    arr[5] = euler[2]
    return arr


def vector2Array(vec):
    return np.array([vec.x, vec.y, vec.z])


def angle_correction(cur_ang, prev_ang):
    correctedAng = np.zeros(len(cur_ang))
    for i in range(len(cur_ang)):
        if (prev_ang[i] >= 0.):
            ang_range = np.floor(prev_ang[i]/np.pi)
            if (cur_ang[i] >= 0):
                correctedAng[i] = cur_ang[i] + ang_range*np.pi
            else:
                correctedAng[i] = cur_ang[i] + (ang_range+1)*np.pi
        elif(prev_ang[i] < 0.):
            ang_range =  np.ceil(prev_ang[i]/np.pi)
            if (cur_ang[i] > 0):
                correctedAng[i] = cur_ang[i] + (ang_range-1)*np.pi
            else:
                correctedAng[i] = cur_ang[i] + ang_range*np.pi
    return correctedAng

def remap_angle(angle): 
#To Map Any angle to an angle between 0 to 2pi
    if (angle >= 0.):
        ang_range = np.floor(angle/(2*np.pi))
        rem_ang = angle - ang_range*(2*np.pi)
    else:
        ang_range =  np.ceil(angle/(2*np.pi)) - 1.
        rem_ang = angle - ang_range*(2*np.pi)
    return rem_ang, ang_range


def reduce_dof(val):
    return val[:2]

def add_dof(val):
    res =  np.zeros(6)
    res[:2] = deepcopy(val[:2])
    res[2] = 0.005
    res[3] = 0.785
    res[4] = -1.57 #- 0.5236 # To force orientation to be gripper facing down deflection of -30 deg
    res[5] = -2.35 # To force orientation for full peg to be right
    return res

def add_dof_mod(val):
    res =  np.zeros(6)
    res[:3] = deepcopy(val[:3])
    res[3] = 0.785
    res[4] = -1.57 #- 0.5236 # To force orientation to be gripper facing down deflection of -30 deg
    res[5] = -2.35 # To force orientation for full peg to be right
    return res


### Math utils
def multilinspace(x1, x2, num=10, endpoint=True):
    pts = np.zeros((len(x1), num))
    newpts = []
    for i in range(len(x1)):
        pts[i,:] = np.linspace(x1[i], x2[i], num=num, endpoint=endpoint)
    for j in range(num):
        newpts.append(np.array([pts[i,j] for i in range(len(x1))]))
    return newpts # returns poses


def find_nearest_idx(array,value):
    # idx = (np.abs(array-value)).argmin()
    idx = (np.linalg.norm(array-value, axis=1)).argmin()
    return array[idx,:], np.linalg.norm(array[idx,:]-value)

    # idx = np.searchsorted(array, value, side="left")
    # raw_input('Step 3, Nearest idx found')
    # if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
    #   return array[idx-1]
    # else:
    #   return array[idx]


# def find_nearest_idx(A, target):
    # #A must be sorted
    # idx = A.searchsorted(target)
    # idx = np.clip(idx, 1, len(A)-1)
    # left = A[idx-1]
    # right = A[idx]
    # idx -= target - left < right - target
    # return idx
