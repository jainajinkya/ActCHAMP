#!/usr/bin/env python

import time
import numpy as np
import rospy
import rospkg
import tf
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers

pos1 = []
ori1 = []
ori1_euler = []
pos2 = []
ori2 = []
ori2_euler = []
diff_pos = []
diff_ori = []
diff_ori_euler = []

start_time = time.time()

def find_stats():
    pos1_mean = np.mean(np.array(pos1), axis=0)
    ori1_mean = np.mean(np.array(ori1), axis=0)
    ori1_euler_mean = np.mean(np.array(ori1_euler), axis=0)

    pos2_mean = np.mean(np.array(pos2), axis=0)
    ori2_mean = np.mean(np.array(ori2), axis=0)
    ori2_euler_mean = np.mean(np.array(ori2_euler), axis=0)

    diff_pos_mean = np.mean(np.array(diff_pos), axis=0)
    diff_ori_mean = np.mean(np.array(diff_ori), axis=0)
    diff_ori_euler_mean = np.mean(np.array(diff_ori_euler), axis=0)  

    pos1_std = np.std(np.array(pos1), axis=0)
    ori1_std = np.std(np.array(ori1), axis=0)
    ori1_euler_std = np.std(np.array(ori1_euler), axis=0)

    pos2_std = np.std(np.array(pos2), axis=0)
    ori2_std = np.std(np.array(ori2), axis=0)
    ori2_euler_std = np.std(np.array(ori2_euler), axis=0)

    diff_pos_std = np.std(np.array(diff_pos), axis=0)
    diff_ori_std = np.std(np.array(diff_ori), axis=0)
    diff_ori_euler_std = np.std(np.array(diff_ori_euler), axis=0)  


    print "pos1: mean = ", pos1_mean, "\n std: ", pos1_std
    print "ori1: mean = ", ori1_mean, "\n std: ", ori1_std, "\n\n"
    print "ori1 Euler: mean = ", ori1_euler_mean, "\n std: ", ori1_euler_std, "\n\n"


    print "pos2: mean = ", pos2_mean, "\n std: ", pos2_std
    print "ori2: mean = ", ori2_mean, "\n std: ", ori2_std,  "\n\n"
    print "ori2 Euler: mean = ", ori2_euler_mean, "\n std: ", ori2_euler_std,  "\n\n"


    print "diff_pos: mean = ", diff_pos_mean, "\n std: ", diff_pos_std
    print "diff_ori: mean = ", diff_ori_mean, "\n std: ", diff_ori_std
    print "diff_ori_euler: mean = ", diff_ori_euler_mean, "\n std: ", diff_ori_euler_std, "\n\n"

    rospack = rospkg.RosPack()
    filepath = rospack.get_path('experiments') + "/data/ar_Tag_error_params.txt"
    f = open(filepath, "w")
    f.write("diff_pos: mean = " + str(diff_pos_mean) + "\n" + "std= " + str(diff_pos_std) + "\n\n")
    f.write("diff_ori: mean = " + str(diff_ori_mean) + "\n" + "std = " + str(diff_ori_std) + "\n\n")
    f.write("In Euler: mean = " + str(diff_ori_euler_mean) + "\n" + "std = " + str(diff_ori_euler_std) + "\n\n")
    f.close()

def callback(msg):
    if(len(msg.markers) > 0):
        pos1.append(np.array([msg.markers[0].pose.pose.position.x, msg.markers[0].pose.pose.position.y, msg.markers[0].pose.pose.position.z]))
        ori1.append(np.array([msg.markers[0].pose.pose.orientation.x, msg.markers[0].pose.pose.orientation.y, msg.markers[0].pose.pose.orientation.z, msg.markers[0].pose.pose.orientation.w]))

        pos2.append(np.array([msg.markers[1].pose.pose.position.x, msg.markers[1].pose.pose.position.y, msg.markers[1].pose.pose.position.z]))
        ori2.append(np.array([msg.markers[1].pose.pose.orientation.x, msg.markers[1].pose.pose.orientation.y, msg.markers[1].pose.pose.orientation.z, msg.markers[1].pose.pose.orientation.w]))

        diff_pos.append(np.array([msg.markers[0].pose.pose.position.x - msg.markers[1].pose.pose.position.x, msg.markers[0].pose.pose.position.y - msg.markers[1].pose.pose.position.y, msg.markers[0].pose.pose.position.z - msg.markers[1].pose.pose.position.z]))

        q1 = np.array([msg.markers[0].pose.pose.orientation.x, msg.markers[0].pose.pose.orientation.y, msg.markers[0].pose.pose.orientation.z, msg.markers[0].pose.pose.orientation.w])
        eu1  = tf.transformations.euler_from_quaternion(q1)
        q2 = np.array([msg.markers[1].pose.pose.orientation.x, msg.markers[1].pose.pose.orientation.y, msg.markers[1].pose.pose.orientation.z, msg.markers[1].pose.pose.orientation.w])
        eu2  = tf.transformations.euler_from_quaternion(q2)

        ori1_euler.append(eu1)
        ori2_euler.append(eu2)
        diff_ori.append(tf.transformations.quaternion_from_euler(eu1[0] - eu2[0], eu1[1] - eu2[1], eu1[2] - eu2[2]))
        diff_ori_euler.append(np.array([eu1[0] - eu2[0], eu1[1] - eu2[1], eu1[2] - eu2[2]]))


    run_duration = time.time() - start_time

    if run_duration > 120.: # time in seconds
        find_stats()
        rospy.signal_shutdown('Killing the node')


if __name__ == "__main__":
    rospy.init_node("ar_detection_error", anonymous=True)
    print "Started Node to calculate ar_detection_error"
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()
