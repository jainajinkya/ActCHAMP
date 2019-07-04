#! /usr/bin/env python

import numpy as np
import rospy
import cPickle as pickle
import sys


###### LOAD DATA
def get_ground_truth(g_name):
    gt_file = open(g_name, 'r')
    gt_data = pickle.load(gt_file)
    gt_file.close()
    return gt_data

def get_test_data(filename='../data/sim_data/articTest_simulated_test_data.txt'):
    test_file =  open(filename, 'r')
    test_data = pickle.load(test_file)
    test_file.close()
    return test_data 

##### COMPARING WITH GROUND TRUTH
def check_n_Segments(data1, data2, datafile):
    print "Segments in Data 1: ", len(data1)
    print "Segments in Data 2: ", len(data2)
    print
    datafile.write("Segments in Data 1:" + str(len(data1)) + "\n")
    datafile.write("Segments in Data 2:" + str(len(data2)) + "\n\n")


def check_segment_lengths(gt_data, test_data, datafile):
    print "Ground Truth Data --> Segment Start: ", gt_data['Start'], "End: ", gt_data['End'], "Length: ", gt_data['Length']
    print "Test Data --> Segment Start: ", test_data['Start'], "End: ", test_data['End'], "Length: ", test_data['Length']
    print "Error in segment Length: ", gt_data['Length'] - test_data['Length']
    print

    datafile.write("Ground Truth Data --> Segment Start: " + str(gt_data['Start']) + ", End: " + str(gt_data['End']) + ", Length: " + str(gt_data['Length']) + "\n")
    datafile.write("Test Data --> Segment Start: " + str(test_data['Start']) + ", End: " + str(test_data['End']) + ", Length: " + str(test_data['Length']) + "\n")
    datafile.write("Error in segment Length: " + str(gt_data['Length'] - test_data['Length']) + "\n\n")



def rotational_model_error(gt_params, test_params, datafile):
    origin_error = np.array([gt_params['rot_center.x'] - test_params['rot_center.x'], gt_params['rot_center.y'] - test_params['rot_center.y'], gt_params['rot_center.z'] - test_params['rot_center.z']])
    origin_orientation_error = np.array([gt_params['rot_orientation.x'] - test_params['rot_orientation.x'],gt_params['rot_orientation.y'] - test_params['rot_orientation.y'], gt_params['rot_orientation.z'] - test_params['rot_orientation.z'], gt_params['rot_orientation.w'] - test_params['rot_orientation.w']])
    rad_error = np.array([gt_params['rot_radius'] - test_params['rot_radius']])
    axis_error = np.array([gt_params['rot_axis.x'] - test_params['rot_axis.x'], gt_params['rot_axis.y'] - test_params['rot_axis.y'], gt_params['rot_axis.z'] - test_params['rot_axis.z'], gt_params['rot_axis.w'] - test_params['rot_axis.w']])

    print "Error in origin:", origin_error, "\tNorm: ", np.linalg.norm(origin_error)
    print "Error in origin_orientation:", origin_orientation_error
    print "Error in Radius:", rad_error
    print "Error in Axis:", axis_error

    datafile.write("Error in origin:" + str(origin_error) + "\tNorm: " + str(np.linalg.norm(origin_error)) + "\n")
    datafile.write("Error in origin_orientation:" + str(origin_orientation_error) + "\n")
    datafile.write("Error in Radius:"+ str(rad_error) + "\n")
    datafile.write("Error in Axis:" + str(axis_error) + "\n\n")



def prismatic_model_error(gt_params, test_params, datafile):
    origin_error = np.array([gt_params['rigid_position.x'] - test_params['rigid_position.x'], gt_params['rigid_position.y'] - test_params['rigid_position.y'], gt_params['rigid_position.z'] - test_params['rigid_position.z']])
    origin_orientation_error = np.array([gt_params['rigid_orientation.x'] - test_params['rigid_orientation.x'],gt_params['rigid_orientation.y'] - test_params['rigid_orientation.y'], gt_params['rigid_orientation.z'] - test_params['rigid_orientation.z'], gt_params['rigid_orientation.w'] - test_params['rigid_orientation.w']])
    axis_error = np.array([gt_params['prismatic_dir.x'] - test_params['prismatic_dir.x'], gt_params['prismatic_dir.y'] - test_params['prismatic_dir.y'], gt_params['prismatic_dir.z'] - test_params['prismatic_dir.z']])

    print "Error in origin:", origin_error, "\tNorm: ", np.linalg.norm(origin_error)
    print "Error in origin_orientation:", origin_orientation_error
    print "Error in Axis:", axis_error

    datafile.write("Error in origin:" + str(origin_error) + "\tNorm: " + str(np.linalg.norm(origin_error)) + "\n")
    datafile.write("Error in origin_orientation:" + str(origin_orientation_error) + "\n")
    datafile.write("Error in Axis:" + str(axis_error) + "\n\n")



def rigid_model_error(gt_params, test_params, datafile):
    position_error = np.array([gt_params['rigid_position.x'] - test_params['rigid_position.x'], gt_params['rigid_position.y'] - test_params['rigid_position.y'], gt_params['rigid_position.z'] - test_params['rigid_position.z']])
    origin_orientation_error = np.array([gt_params['rigid_orientation.x'] - test_params['rigid_orientation.x'],gt_params['rigid_orientation.y'] - test_params['rigid_orientation.y'], gt_params['rigid_orientation.z'] - test_params['rigid_orientation.z'], gt_params['rigid_orientation.w'] - test_params['rigid_orientation.w']])


    print "Error in Position:", position_error, "\tNorm: ", np.linalg.norm(position_error)
    print "Error in Orientation:", origin_orientation_error

    datafile.write("Error in position:" + str(position_error) + "\tNorm: " + str(np.linalg.norm(position_error)) + "\n")
    datafile.write("Error in origin_orientation:" + str(origin_orientation_error) + "\n\n")



def error_model_parameters(gt_data, test_data, datafile):
    if gt_data['Model'] == test_data['Model']:
        gt_params = gt_data['Data']['params']
        test_params = test_data['Data']['params']

        if gt_data['Model'] == 'rotational':
            rotational_model_error(gt_params, test_params, datafile)
        elif gt_data['Model'] == 'prismatic':
            prismatic_model_error(gt_params, test_params, datafile)
        elif gt_data['Model'] == 'rigid':
            rigid_model_error(gt_params, test_params, datafile)

    else:
        print "Please pass segments having same model type"


def compare_data_with_GT(gt_data, test_data, datafile, run=1):
    print "###################################"
    print "Difference with Ground Truth for Dataset: ", run
    print "###################################"

    datafile.write("###################################\n")
    datafile.write("Difference with Ground Truth for Dataset: " + str(run) + "\n")
    datafile.write("###################################\n")

    check_n_Segments(gt_data, test_data, datafile)
    gt_data_model_names = [ml['Model'] for ml in gt_data]

    for m1 in test_data:
        print "Model in test data: ", m1['Model']
        datafile.write("----------------------------------------\n")
        datafile.write("Model in test data: " +  str(m1['Model']) + "\n")

        if m1['Model'] in gt_data_model_names:
            idx = gt_data_model_names.index(m1['Model'])
            check_segment_lengths(gt_data[idx], m1, datafile)
            error_model_parameters(gt_data[idx], m1, datafile)
            print "\n\n"
        else:
            print "Detection mismatch. Couldn't find a matching model in Ground Truth"
            datafile.write("Detection mismatch. Couldn't find a matching model in Ground Truth \n\n")



##### COMPARING WITH/WITHOUT ACTION
def diff_log_likelihood(data1, data2, datafile):
    print "log_likelihood for Data 1: ", data1['Data']['log_likelihood']
    print "log_likelihood for Data 2: ", data2['Data']['log_likelihood']
    datafile.write("log_likelihood for Data 1: " + str(data1['Data']['log_likelihood']) + "\n")
    datafile.write("log_likelihood for Data 2: " + str(data2['Data']['log_likelihood']) + "\n")

    err = data1['Data']['log_likelihood'] - data2['Data']['log_likelihood']
    print "Difference in log_likelihood: ", err
    datafile.write("Differences in log_likelihood: " + str(err) + "\n\n")

    return err

def diff_model_evidence(data1, data2, datafile):
    print "Model Evidence for Data 1: ", data1['Data']['model_evidence']
    print "Model Evidence for Data 2: ", data2['Data']['model_evidence']
    datafile.write("Model Evidence for Data 1: " + str(data1['Data']['model_evidence']) + "\n")
    datafile.write("Model Evidence for Data 2: " + str(data2['Data']['model_evidence']) + "\n")

    err = data1['Data']['model_evidence'] - data2['Data']['model_evidence']
    print "Model Evidence: ", err
    datafile.write("Differences in Model evidences: " + str(err) + "\n\n")

    return err

def compare_data_with_other(test_data1, test_data2, datafile):
    print "Differences in the two Approaches:\n"

    datafile.write("###################################\n")
    datafile.write("Difference in Two Approaches \n")
    datafile.write("###################################\n")

    for i in range(len(test_data1)):
        print "For ", i, "-th Model:"
        datafile.write("For " + str(i) + "-th Model: \n")
        diff_log = diff_log_likelihood(test_data1[i], test_data2[i], datafile)
        diff_me = diff_model_evidence(test_data1[i], test_data2[i], datafile)
        print
        datafile.write("----------------------------------------\n")


if __name__ == "__main__":
    g_name = '../data/simulations/sim_data/' + str(sys.argv[1]) +'_groundTruth.txt'
    gt_data = get_ground_truth(g_name)

    t1_name = '../data/simulations/cp_data/cpData_' + str(sys.argv[1]) +'_no_action.txt'
    test_data1 = get_test_data(t1_name)

    t2_name = '../data/simulations/cp_data/cpData_' + str(sys.argv[1]) +'_with_action.txt'
    test_data2 = get_test_data(t2_name)

    # print "Ground Truth data :\n", gt_data
    # print "\n\n\n Test Data: \n", test_data
    f_name = "../data/simulations/error_data/error_analysis_" + str(sys.argv[1])  +".txt"

    datafile = open(f_name, 'w')
    compare_data_with_GT(gt_data, test_data1, datafile, 1)
    compare_data_with_GT(gt_data, test_data2, datafile, 2)

    compare_data_with_other(test_data1, test_data2, datafile)

    datafile.close()



















