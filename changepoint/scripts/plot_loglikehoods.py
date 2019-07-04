#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import csv
import sys

def item_value_extractor(val, delimiter=':'):
    val_dict = {}
    print val
    val_dict["time"] = int(val[0])
    for q in val[1:]:
        spt = q.split(':')
        if len(spt) > 0:
            val_dict[spt[0]] = float(spt[1])
    
    return val_dict

def dict_to_array(dict_array):
    # prismatic: column 0, rigid: column 1, rotational: column 2
    b_array = []

    for d in dict_array:
        s_array = []
        keys = sorted(d.keys())
        for key in keys:
            s_array.append(d[key])
        b_array.append(s_array)

    unsorted_arr = np.array(b_array)
    # print "unsorted_arr: ", unsorted_arr
    sorted_arr = unsorted_arr[unsorted_arr[:,-1].argsort()]
    # print "sorted_arr: ", sorted_arr
    return sorted_arr[:, :3]

def plot_lls(data):
    f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    min_seg_length = 0

    ### Reducing champ dataset by one time stamp for fair comparison
    # data[0] = np.delete(data[0], -1, 0)

    # timesteps = range(min_seg_length, len(data[0])+ min_seg_length)
    timesteps = range(len(data[0]))
    len_diff = len(data[0]) - len(data[1])
    print "Length of Data set 0: ", len(data[0]) 
    print "Length of Data set 1: ", len(data[1])
    print "len_diff = ", len_diff


    # print "Time steps", timesteps
    tick_freq = 10   


    # Plot 1
    l1, = ax1.plot(timesteps, data[0][:,0], 'b')
    l2, = ax1.plot(timesteps, data[0][:,1], 'k')
    l3, = ax1.plot(timesteps, data[0][:,2], 'r')

    ax1.legend([l1, l2, l3], ['prismatic', 'rigid', 'rotational'], loc = 'lower right')
    ax1.xaxis.grid(True)
    ax1.set_title('CHAMP')
    # ax1.set_xticks(np.arange(min_seg_length, len(data[0])+ min_seg_length,  2))
    ax1.set_xticks(np.arange(0, len(data[0]),  tick_freq))

    # Plot 2
    # timesteps = range(min_seg_length, len(data[1])+ min_seg_length)
    timesteps = range(len(data[1]))

    l4, = ax2.plot(timesteps, data[1][:,0], 'b')
    l5, = ax2.plot(timesteps, data[1][:,1], 'k')
    l6, = ax2.plot(timesteps, data[1][:,2], 'r')

    ax2.legend([l4, l5, l6], ['prismatic', 'rigid', 'rotational'], loc = 'lower right')
    ax2.xaxis.grid(True)
    ax2.set_title('Act-CHAMP')
    # ax2.set_xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), 2))
    ax2.set_xticks(np.arange(0, len(data[1])+1, tick_freq))


    # Comparison Plot
    f1, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    timesteps = range(min_seg_length, len(data[1])+ min_seg_length)

    # Subplot 1
    if len_diff > 0:    
        l1, = ax1.plot(timesteps, data[0][:-len_diff,0], 'b')
    else:
        l1, = ax1.plot(timesteps, data[0][:,0], 'b')

    l2, = ax1.plot(timesteps, data[1][:,0], 'r')

    ax1.legend([l1, l2], ['CHAMP', 'Action'], loc = 'lower right')
    ax1.xaxis.grid(True)
    ax1.set_title('Comparison Plot for Prismatic Model')
    ax1.set_xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), tick_freq))

    #Subplot 2
    if len_diff > 0:    
        l3, = ax2.plot(timesteps, data[0][:-len_diff,1], 'b')
    else:
        l3, = ax2.plot(timesteps, data[0][:,1], 'b')

    l4, = ax2.plot(timesteps, data[1][:,1], 'r')

    ax2.legend([l3, l4], ['CHAMP', 'Action'], loc = 'lower right')
    ax2.xaxis.grid(True)
    ax2.set_title('Comparison Plot for Rigid Model')
    ax2.set_xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), tick_freq))


    #Subplot 3
    if len_diff > 0:    
        l5, = ax3.plot(timesteps, data[0][:-len_diff,2], 'b')
    else:
        l5, = ax3.plot(timesteps, data[0][:,2], 'b')

    l6, = ax3.plot(timesteps, data[1][:,2], 'r')

    ax3.legend([l5, l6], ['CHAMP', 'Action'], loc = 'lower right')
    ax3.xaxis.grid(True)
    ax3.set_title('Comparison Plot for Rotational Model')
    ax3.set_xticks(np.arange(min_seg_length, len(data[1]+ 1 + min_seg_length), tick_freq))


    ## Individual Plots
    f2 = plt.figure()
    l1, = plt.plot(timesteps, data[1][:,0], 'b')
    l2, = plt.plot(timesteps, data[1][:,2], 'r')
    plt.legend([l1, l2], ['Prismatic', 'Rotational'], loc = 'lower right')
    ax = plt.gca()
    ax.xaxis.grid(True)
    plt.title('Active Champ Model')
    plt.xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), tick_freq))

    f2 = plt.figure()
    if len_diff > 0:    
        l1, = plt.plot(timesteps, data[0][:-len_diff,0], 'b')
    else:
        l1, = plt.plot(timesteps, data[0][:,0], 'b')

    if len_diff > 0:    
        l2, = plt.plot(timesteps, data[0][:-len_diff,2], 'r')
    else:
        l2, = plt.plot(timesteps, data[0][:,2], 'r')

    plt.legend([l1, l2], ['Prismatic', 'Rotational'], loc = 'lower right')
    ax = plt.gca()
    ax.xaxis.grid(True)
    plt.title('Champ Model')
    plt.xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), tick_freq))

    f3 = plt.figure()
    if len_diff > 0:    
        l1, = plt.plot(timesteps, data[0][:-len_diff,0], 'b')
    else:
        l1, = plt.plot(timesteps, data[0][:,0], 'b')

    if len_diff > 0:    
        l2, = plt.plot(timesteps, data[0][:-len_diff,2], 'r')
    else:
        l2, = plt.plot(timesteps, data[0][:,2], 'r')

    l3, = plt.plot(timesteps, data[1][:,0], 'b--')
    l4, = plt.plot(timesteps, data[1][:,2], 'r--')
    plt.legend([l1, l2, l3, l4], ['CH: Prism', 'CH: Rot', 'AC: Prism', 'AC: Rot'], loc = 'lower right')
    ax = plt.gca()
    ax.xaxis.grid(True)
    plt.title('Compare Rotational v/s Prismatic')
    plt.xticks(np.arange(min_seg_length, len(data[1]+ min_seg_length), tick_freq))


    plt.show()
    # raw_input('')


def plot_datafile(filenames):
    arrs = []
    for filename in filenames: 
        with open(filename, 'rb') as csvfile:
            filereader = csv.reader(csvfile, delimiter=',')
            vals = []
            for line in filereader:
                line.remove('')
                vals.append(item_value_extractor(line))

            arrs.append(dict_to_array(vals))
        
    plot_lls(arrs)


def main():
    filename1 = "../test/data/loglikelihoods/" + sys.argv[1] + "_champ.csv"
    filename2 = "../test/data/loglikelihoods/"+ sys.argv[1] + "_action.csv"
    plot_datafile([filename1, filename2])



if __name__ == "__main__":
    main()