#!/usr/bin/env python

import rospy
import rosbag
import cPickle as pickle
from changepoint.msg import DataPoint

def main():
    # Open pkl file
    # f_name = "microwave_data/mw_run_108_filtered_original" ## microwave no grip
    # f_name = "old_data/microwave_run6_filtered_original" ## microwave with grip
    # f_name = "drawer_data/drawer_run3_filtered_original" ## drawer without grip
    # f_name = "old_data/drawer_openClose_3_filtered_original"  ## Drawer with grip
    f_name = "old_data/drawer_openClose_3_close_only_filtered_original"  ## Drawer with grip


    in_file = "../../experiments/data/pkl_files/" + f_name + ".pkl"
    out_bag = "../../experiments/data/bagfiles/processed/" + f_name + ".bag"

    [pose_data, action_data] = pickle.load(open(in_file, "r"))

    # Save in the rosbag file
    i = 1
    with rosbag.Bag(out_bag, 'w') as outbag:
        for pose, act in zip(pose_data, action_data):
            t = rospy.Time(i)
            outbag.write("changepoint/processed_pose_data", DataPoint(pose.tolist()), t)
            outbag.write("changepoint/processed_action_data", DataPoint(act.tolist()), t)
            i += 1


if __name__ == "__main__":
    main()
