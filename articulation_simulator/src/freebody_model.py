#!/usr/bin/env python

import rospy
import sys, time
import numpy as np
import numpy.linalg as la
from copy import deepcopy
from utils import *



class Freebody_model(object):
    """docstring for Rotational_model"""
    def __init__(self):
        super(Freebody_model, self).__init__()
        self.origin = np.array([0., 0., 0.])
        self.origin_orientation = np.array([0., 0., 0.])

        #Parameters
        self.Int_res = 100
        self.Data_len = 100

            
    def print_attributes(self, arg=False):
        if arg:
            ## Printing Stuff
            print "Freebody Model Attributes:"
            print "origin : ", self.origin


    def generate_actions(self):
        arr = np.random.uniform(-20.,20., (self.Data_len, 3))
        # arr = np.zeros((self.Data_len, 3))
        # arr[:,0] = 5.0
        return arr


    def simulate_one_step(self, pos, action):
        new_pos = pos + action
        return new_pos


    def visualize(self, data, do_plot=False, do_animate=False):
        pos_set = data[0]
        action_set = data[1]

        # Calculations for plotting
        # Actions
        action_plot_set = []
        for i in range(self.Data_len):
            action_plot_set.append(pos_set[i] + 1.0*action_set[i])

        action_plot_set = np.array(action_plot_set)


        if do_plot:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            # ax.plot(pos_set[:,0], pos_set[:,1], pos_set[:,2])
            ax.scatter(pos_set[:,0], pos_set[:,1], pos_set[:,2], s=100)

            #Center
            ax.scatter(self.origin[0], self.origin[1], self.origin[2], s=100, c='k', )

            # Actions
            for i, pt in enumerate(action_plot_set):
                # ax.plot([pos_set[i,0], pt[0]], [pos_set[i,1], pt[1]], [pos_set[i,2], pt[2]], 'g')
                a = Arrow3D([pos_set[i,0], pt[0]], [pos_set[i,1], pt[1]], [pos_set[i,2], pt[2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="g")
                ax.add_artist(a)

            # ax.axis('equal')
            ax.set_xlim(-100., 100.)
            ax.set_xlabel('X')
            ax.set_ylim(-100., 100.)
            ax.set_ylabel('Y')
            ax.set_zlim(-100., 100.)
            ax.set_zlabel('Z')

            ax.set_title('Simulator for Freebody_model')

        # Animation

        if do_animate:
            fig2 = plt.figure()
            ax2 = Axes3D(fig2)

            a = Arrow3D([pos_set[0,0], action_plot_set[0,0]], [pos_set[0,1], action_plot_set[0,1]], [pos_set[0,2], action_plot_set[0,2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

            plots = [ax2.plot([pos_set[0,0]], [pos_set[0,1]], [pos_set[0,2]], 'o') , ax2.add_artist(a)]

            #Center
            ax2.scatter(self.origin[0], self.origin[1], self.origin[2], s=50, c='k', )

            ax2.set_xlim(-100., 100.)
            ax2.set_xlabel('X')
            ax2.set_ylim(-100., 100.)
            ax2.set_ylabel('Y')
            ax2.set_zlim(-100., 100.)
            ax2.set_zlabel('Z')

            ax2.set_title('Simulator for Freebody_model')

            # Creating the Animation object
            pos_act_ani = animation.FuncAnimation(fig2, self.update_animation, self.Data_len, fargs=(pos_set, action_plot_set, plots, ax2), interval=500, blit=False)

        plt.show()


    def update_animation(self, num, pos_set, action_plot_set, plots, ax):
        # plots[0].pop(0).remove()
        plots[0] = ax.plot([pos_set[num,0]], [pos_set[num, 1]], [pos_set[num,2]], 'bo', ms=10)

        time.sleep(0.1)
        
        plots[1].remove()
        a = Arrow3D([pos_set[num,0], action_plot_set[num,0]], [pos_set[num,1], action_plot_set[num,1]], [pos_set[num,2], action_plot_set[num,2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
        plots[1] = ax.add_artist(a)

        if num >= len(pos_set)-2:
            raw_input('Press Enter to close Animation')
            sys.exit(1)
        return plots


    def save_data(self, data, filename="Freebody_model.txt"):
        print "Data saved in: ", filename
        data_new = [{'origin': self.origin, 'Positions': data[0], 'Actions' : data[1]}]
        pickle.dump(data_new, open(str(filename), "wb"))



def main():
    fb_model = Freebody_model()
    fb_model.print_attributes(True)

    pos = deepcopy(fb_model.origin) # start at the ref_vector
    pos_set = [pos.T]
    action_set = deepcopy(fb_model.generate_actions())
    # print "Action_set = ", action_set

    for action in action_set:
        pos = deepcopy(fb_model.simulate_one_step(pos, action))
        pos_set.append(pos.T)

    pos_set = np.array(pos_set)
    # print "Set of poses: \n ", pos_set

    do_plot = False
    do_animate = False

    fb_model.visualize([pos_set, action_set], do_plot, do_animate)

    # Save data
    filename = "../data/sim_data/Freebody_data.txt"
    fb_model.save_data([pos_set, action_set], filename)



if __name__ == '__main__':
    main()