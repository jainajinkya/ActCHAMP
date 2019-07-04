#!/usr/bin/env python

import sys
import rosbag
import numpy as np
import cPickle as pickle
from scipy.interpolate import interp1d
from scipy.signal import butter, filtfilt, medfilt
from utils import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
import tf.transformations as tr

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    # return array[idx]
    return idx



def smooth(x):
    b, a = butter(2, 0.125)
    return filtfilt(b, a, x, axis=0, method='gust')


def resample_interp(x, t, min_t, max_t, hz):
    # f = interp1d(t, x, kind='linear', axis=0)
    f = interp1d(t, x, kind='nearest', axis=0)
    t_rs = np.arange(min_t, max_t, 1.0/hz)
    x_rs = f(t_rs)
    return x_rs, t_rs

# def resample_interp_array(arr, t, min_t, max_t, hz):
#     arr = np.array(arr)
#     arr_interp = []

#     for i in range(len(arr[0,:])):
#         x_interp, t_interp = resample_interp(arr[:,i], t, min_t, max_t, hz)
#         arr_interp.append(x_interp)

#     return arr_interp, t_interp

def transform_action(trans, act):
    rot = tr.quaternion_matrix(trans[3:])
    hom_act = np.array(act.tolist() + [1.])
    print "rot: ", rot
    print "action: ", hom_act.T
    new_act = rot.dot(hom_act)
    print "rotated action: ",  new_act
    return new_act[:3]
    


def convert_actions_to_local_frame(actions, action_ts, tfs, tfs_ts):
    local_acts = []
    for i, act in enumerate(actions):
        tf_idx = find_nearest(tfs_ts, action_ts[i])
        local_act = transform_action(tfs[tf_idx], act)
        local_acts.append(local_act)
    return local_acts


def threshold_using_ts(min_ts, max_ts, ts_array, arr):
    # This gives index of first item to be included in  thresholded x
    min_idx = next((i for i, x in enumerate(ts_array) if x >= min_ts), -1)
    # this gives index of the last iterm to be stored in thresholded x
    max_idx = next((i for i, x in enumerate(
        ts_array) if x > max_ts), len(ts_array))
    return arr[min_idx:max_idx]


def plot_data(data, title='figure'):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.gca(projection='3d')
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=100)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    # ax.plot(data[:,0], data[:,1], data[:,2])

    X = data[:, 0]
    Y = data[:, 1]
    Z = data[:, 2]
    max_range = np.array(
        [X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max()+X.min()) * 0.5
    mid_y = (Y.max()+Y.min()) * 0.5
    mid_z = (Z.max()+Z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.title(title)
    plt.show()


def filter_outliers(data, ts_data, action_data, action_ts, n_means=2, inliers_fraction=1.25):
    data = np.array(data)
    from scipy.cluster.vq import kmeans as kmeans
    means, sig = kmeans(data, n_means, iter=20, thresh=1e-03, check_finite=True)
    print "means: ", means
    print "sig: ", sig
    mean = np.array(means[0])

    final_data = []
    final_ts = []
    action_inliers = []
    action_ts_inliers = []
    n_pts = 0

    for i, x in enumerate(data):
        if np.linalg.norm(x-mean) < inliers_fraction*sig:
            final_data.append(x)
            final_ts.append(ts_data[i])

            idx = find_nearest(action_ts, ts_data[i])
            action_inliers.append(action_data[idx])
            action_ts_inliers.append(action_ts[idx])
            n_pts += 1

    print "Retaining points: ", n_pts
    # print "final_ts: ", final_ts
    return np.array(final_data), np.array(final_ts),  \
        np.array(action_inliers), np.array(action_ts_inliers)

def filter_outliers2(data, ts_data, action_data, action_ts):
    thres = 0.1
    inliers = []
    inliers_ts = []
    action_inliers = []
    action_ts_inliers = []

    for i, p in enumerate(data):
        # if not(p[0] > 0.05 or p[0] < 0. or p[1] > -0.12 or p[2] > 0.20): #mw_run_105
        # if p[0] > 0.07 and p[0] < 0.1: # d_run_101
        # if not(p[0] > 0.05 or p[0] < 0. or p[1] > -0.12 ): #mw_run_108
        # if not(p[0] < -0.015): #stapler_4
        if not(p[2] > 0.): #stapler_108

            inliers.append(p)
            inliers_ts.append(ts_data[i])

            idx = find_nearest(action_ts, ts_data[i])
            action_inliers.append(action_data[idx])
            action_ts_inliers.append(action_ts[idx])

    return np.array(inliers), np.array(inliers_ts), \
        np.array(action_inliers), np.array(action_ts_inliers)


def filter_outliers3(data, ts_data, action_data, action_ts):
    thres = 0.1
    inliers = []
    inliers_ts = []
    action_inliers = []
    action_ts_inliers = []

    for i in range(len(data)-1):
        p = data[i]
        q = data[i+1]
        delta = np.array(p) - np.array(q)
        if (delta[1] > 0.02 and delta[0] > 0.01): #stapler_104
            continue
        else:
            inliers.append(p)
            inliers_ts.append(ts_data[i])

            idx = find_nearest(action_ts, ts_data[i])
            action_inliers.append(action_data[idx])
            action_ts_inliers.append(action_ts[idx])

    return np.array(inliers), np.array(inliers_ts), \
        np.array(action_inliers), np.array(action_ts_inliers)




if __name__ == "__main__":

    resample_freq = 18

    # if len(sys.argv) > 1:

    pose_data = []
    pose_ts = []
    action_data = []
    action_ts = []
    local_tfs = []
    local_tfs_ts = []

    # Table leg Screw insert run1
    # bagFile = "old_data/table_assembly_run1_unscrew"  
    # # thres = [1526429507.0, 1526429657.0]
    # # thres = [1526429505.0, 1526429660.0]
    # thres = [0., 1600000000.]


    # microwave_run5 :
    # thres = [1526068437.,1526068472.]

    # microwave_run6:
    # bagFile = "old_data/microwave_run6"
    # thres = [1526068630., 1526068666.0]

    # mw_run_101:
    # bagFile = "microwave_data/mw_run_101"
    # thres = [1544902640., 1544902685]

    # ## mw_run_105 No grip
    # bagFile = "microwave_data/mw_run_105"
    # thres = [1545161086., 1545161126.]

    # # mw_run_106 no grip
    # bagFile = "microwave_data/mw_run_106"
    # thres = [1545167830., 1545167887.]

    # ## mw_run_108 No grip
    # bagFile = "microwave_data/mw_run_108"
    # thres = [1545350127., 1545350160.]

    # ## Static Dataset
    # bagFile = "microwave_data/mw_run_104"
    # thres = [1545158755.5, 1545158763.5]

    # ## Drawer_run3 without grip
    # bagFile = "drawer_data/drawer_run3"
    # thres = [1538509703.6, 1538509763.4]

    # drawer without grip 3
    # bagFile = "drawer_data/drawer_run3"
    # thres = [1538509703.6, 1538509711.6] ## small
    # thres = [1538509720.6, 1538509726.6]  ## small 2
    # thres = [1538509731.0, 1538509737.0]  ## small 3
    # thres = [1538509758.1, 1538509764.1]  ## small 4

    # drawer without grip 3 sparse actions
    # bagFile = "drawer_data/drawer_run3"
    # thres = [1538509760., 1538509768.]

    # Drawer run4 without grip
    # bagFile = "drawer_data/drawer_run4"
    # thres = [1538510225., 1538510289.]

    # drawer run6 without grip
    # bagFile = "drawer_data/drawer_run6"
    # thres = [1538603473, 1538603505]

    # drawer with grip
    # bagFile = "old_data/drawer_close_1"
    # thres = [1526667165, 1526667236] 

    # drawer with grip
    # bagFile = "old_data/drawer_open_1"
    # thres = [1526667015, 1526667080]

    ## drawer with grip 3
    # bagFile = "old_data/drawer_openClose_3"
    # thres = [1526667660, 1526667754] # both open close
    # thres = [1526667694, 1526667754] # Just close


    # a_run_101
    # bagFile = "drawer_data/d_run_101"
    # thres = [1545351886., 1545351920.]

    # ## Stapler_4
    # bagFile = "stapler_4"
    # # thres = [1548973289., 1548973320.]
    # thres = [1548973289., 1548973300.]

    # bagFile = "stapler_104"
    # thres = [1549011229., 1549011252.]

    # bagFile = "stapler_105"
    # thres = [1549012662., 1549012686]

    # bagFile = "stapler_106"
    # thres = [1549013240., 1549013285.]

    # bagFile = "stapler_107"
    # thres = [1549014159., 1549014189.]

    # bagFile = "stapler_108"
    # thres = [1549014475., 1549014497.]
    # thres = [1549014475., 1549014492.]

    # bagFile = "stapler_109"
    # thres = [1549016227., 1549016245.]

    # bagFile = "stapler_202"
    # thres = [1551483211., 1551483230.]

    ## Microwave no filter run 001
    # bagFile = "microwave_data_no_marker/run_001"
    # thres = [1557868167., 1557868188.]
    # bagFile = "microwave_data_no_marker/run_002"
    # thres = [1557870392., 1557870410.]

    # bagFile = "microwave_data_no_marker/run_no_grip_001"
    # thres = [1557871534., 1557871557.]
    # bagFile = "microwave_data_no_marker/run_no_grip_002"
    # thres = [1557871943., 1557871959.]

    ## Drawer No marker
    # bagFile = "drawer_no_marker/run_001"
    # thres = [1558040975., 1558040986.]

    ## Stapler No Marker
    # bagFile = "stapler_no_marker/run_001"
    # thres = [1558642010., 1558642018.]

    bagFile = "stapler_no_marker/run_002"
    thres = [1558644127., 1558644135.]

    # ## Stapler_3
    # bagFile = "stapler_3"
    # thres = [1548973047., 1548973061.]

    ## Stapler_2
    # bagFile = "stapler_2"
    # thres = [1548972684., 1548972708.]

    thres_ts_min = thres[0]
    thres_ts_max = thres[1]
    # bag = rosbag.Bag("../data/bagfiles/" + sys.argv[1] + ".bag")
    bag = rosbag.Bag("../data/bagfiles/" + bagFile + ".bag")

    for topic, msg, t in bag.read_messages(topics=['/changepoint/pose_data']):
        pose_ts.append(msg.header.stamp.to_sec())
        pose_data.append(poseStamped2Array(msg, quat2Euler=False))

    pose_data = np.array(pose_data)

    if(len(pose_data) < 1):
        print "UNABLE TO OPEN THE FILE\n\n\n"

    plot_data(pose_data, title="Raw data")


    for topic, msg, t in bag.read_messages(topics=['/changepoint/action_data']):
        action_ts.append(msg.header.stamp.to_sec())
        action_data.append(vector2Array(msg.wrench.force))


    ###########################################
    ######## Convert Actions to local Frame ##############
    ############################################   

    # ## Stapler only
    # for topic, msg, t in bag.read_messages(topics=['/changepoint/reference_frame']):
    #     local_tfs_ts.append(msg.header.stamp.to_sec())
    #     local_tfs.append(poseStamped2Array(msg, quat2Euler=False))


    # action_data = convert_actions_to_local_frame(action_data, action_ts, local_tfs, local_tfs_ts)

    
    ###########################################
    ######## Time Thresholding ##############
    ############################################
    pose_data = copy.copy(np.array(threshold_using_ts(
        thres_ts_min, thres_ts_max, pose_ts, pose_data)))
    pose_ts = copy.copy(threshold_using_ts(
        thres_ts_min, thres_ts_max, pose_ts, pose_ts))

    action_data = np.array(threshold_using_ts(
        thres_ts_min, thres_ts_max, action_ts, action_data))
    action_ts = threshold_using_ts(
        thres_ts_min, thres_ts_max, action_ts, action_ts)

    print "Max allowed size: ", len(pose_data)
    plot_data(np.array(pose_data), title='Time thresholded data')

    ###########################################
    ######## Extra Filters ##############
    ############################################
  
    ## Extra FILTERS
    # ## ONLY FOR DRAWER
    # pose_data, pose_ts, action_data, action_ts = copy.copy(filter_outliers(pose_data, pose_ts, action_data, action_ts))

    # # mw_run_105 or 108
    # pose_data, pose_ts, action_data, action_ts = copy.copy(filter_outliers2(pose_data, pose_ts, action_data, action_ts))

    ## stapler_104
    # pose_data, pose_ts, action_data, action_ts = copy.copy(filter_outliers3(pose_data, pose_ts, action_data, action_ts))

    # plot_data(np.array(pose_data), title='Filtered data')

    # pose_data_resampled = copy.copy(pose_data)
    # action_data_resampled  = copy.copy(action_data)


    ## Reclustering
    # min_vals = np.amin(pose_data_resampled, axis=0)
    # max_vals = np.amax(pose_data_resampled, axis=0)

    # print "File name: ", bagFile
    # print "Spread along x: ", max_vals[0] - min_vals[0]
    # print "Spread along y: ", max_vals[1] - min_vals[1]
    # print "Spread along z: ", max_vals[2] - min_vals[2]


    ###########################################
    ######## RESAMPLING ##############
    ############################################
    
    pose_data_smooth = smooth(pose_data)
    pose_data_smooth = copy.copy(pose_data)
    # action_data_smooth = smooth(action_data)
    action_data_smooth = copy.copy(action_data)
    min_ts = max(pose_ts[0], action_ts[0])
    max_ts = min(pose_ts[-1], action_ts[-1])
    
    pose_data_resampled, ts_pos = resample_interp(
        pose_data_smooth, pose_ts, min_ts, max_ts, resample_freq)
    action_data_resampled, ts_act = resample_interp(
        action_data_smooth, action_ts, min_ts, max_ts, resample_freq)

    print "len pose : ", len(pose_data_resampled), "\t action: ", len(action_data_resampled)
    plot_data(pose_data_resampled, title="resmapled data")

    # # # #### Plots
    # plot_data(np.array(pose_data_resampled), title='Resampled Data')

    # fig = plt.figure()
    # # ts = range(len(pose_data[:,0]))
    # plt.plot(pose_ts, pose_data[:,0], 'b')
    # # ts = range(len(pose_data_resampled[:,0]))
    # plt.plot(ts_pos, pose_data_resampled[:,0], 'r--')

    # fig2 = plt.figure()
    # plt.plot(action_ts, action_data[:,0], 'b')
    # plt.plot(ts_act, action_data_resampled[:,0], 'r--')

    # fig3 = plt.figure()
    # ts = range(len(action_data_resampled[:,0]))
    # action_means = np.linalg.norm(action_data_resampled, axis=1)
    # plt.plot(ts, action_means)

    # fig4 = plt.figure()
    # ax = plt.gca()
    # ts = range(len(action_data[:, 0]))
    # actual_action_means = np.linalg.norm(action_data, axis=1)
    # plt.plot(ts, actual_action_means)

    # actual_action_means_zero_mean = actual_action_means - np.mean(actual_action_means)
    # plt.plot(ts, actual_action_means_zero_mean, 'g')

    # plt.plot(ts, action_data[:, 0], 'r')

    # ax.set_yticks([0.01, 0.02, 0.05], minor=False)
    # ax.xaxis.grid(True)
    # ax.yaxis.grid(True)

    # plt.show()

    # # plot_data(pose_data_smooth)

    # write data to a pkl file
    # f = "../data/pkl_files/" + sys.argv[1] + ".pkl"]
    # outFile_name = bagFile + "_new_filtered"
    outFile_name = bagFile
    f = "../data/pkl_files/" + outFile_name + ".pkl"
    # f = "../data/pkl_files/" + "static_data" + ".pkl"
    # f = "../data/pkl_files/" + "drawer_close_1" + ".pkl"

    pickle.dump([pose_data_resampled, action_data_resampled], open(f, "w"))

    print "Converted rosbag file:  " + bagFile + "\nTo pkl file: " + f
