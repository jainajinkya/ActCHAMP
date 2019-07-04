#!/usr/bin/env python

import rospy
import sys, time
import numpy as np
import numpy.linalg as la
from copy import deepcopy
from utils import *


class Rotational_model(object):
    """docstring for Rotational_model"""
    def __init__(self):
        super(Rotational_model, self).__init__()
        self.origin = np.array([0., 0., 0.])
        self.rad = 10.
        self.axis = np.array([0., 0., 1.])
        self.axis /= la.norm(self.axis)
        self.ref_vector = deepcopy(self.origin + self.rad*get_perpendicular_vector(self.axis))  # To measure angle. Assuming it is the start vector

        # Parameters
        self.Int_res = 100
        self.Data_len = 100
            
    def print_attributes(self, arg=False):
        if arg:
            ## Printing Stuff
            print "Rotational Model Attributes:"
            print "Axis : ", self.axis
            print "origin : ", self.origin
            print "Radius : ", self.rad
            print "Reference Vector : ", self.ref_vector

    def generate_actions(self):
        # arr = 2.*np.random.rand(self.Data_len, 3)
        arr = np.zeros((self.Data_len, 3))
        arr[:,0] = 1.0
        return arr


    def simulate_one_step(self, pos, action):
        theta = 0.
        r_vector = deepcopy(pos - self.origin)

        for i in range(self.Int_res): # Numerical Interation of 1000 smaller time steps
            # Find Tangential vector
            v_t = deepcopy(np.cross(self.axis, r_vector/self.rad))

            # Find Angular Velocity
            w_t = deepcopy(np.dot(action, v_t)/self.rad)

            # Find  delta theta
            theta += deepcopy(w_t*(1./self.Int_res))
        
        Rot_mat = deepcopy(rotation_matrix(self.axis, theta))
        new_pos = self.origin + np.dot(Rot_mat, r_vector)
        return new_pos


    def visualize(self, data, do_plot=False, do_animate=False):
        pos_set = data[0]
        action_set = data[1]

        # Calculations for plotting
        # Axis
        center_line = []
        for i in np.arange(-30., 30., 0.5):
            center_line.append((self.origin + i*self.axis).T)
        
        center_line = np.array(center_line)

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
            ax.scatter(self.origin[0], self.origin[1], self.origin[2], s=200, c='k', )

            # Axis
            ax.plot(center_line[:,0], center_line[:,1], center_line[:,2], 'k--')

            # Actions
            for i, pt in enumerate(action_plot_set):
                # ax.plot([pos_set[i,0], pt[0]], [pos_set[i,1], pt[1]], [pos_set[i,2], pt[2]], 'g')
                a = Arrow3D([pos_set[i,0], pt[0]], [pos_set[i,1], pt[1]], [pos_set[i,2], pt[2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="g")
                ax.add_artist(a)

            # ax.axis('equal')
            ax.set_xlim(-10., 10.)
            ax.set_xlabel('X')
            ax.set_ylim(-10., 10.)
            ax.set_ylabel('Y')
            ax.set_zlim(-10., 10.)
            ax.set_zlabel('Z')

            ax.set_title('Simulator for Rotational_model')

        # Animation

        if do_animate:
            fig2 = plt.figure()
            ax2 = Axes3D(fig2)

            a = Arrow3D([pos_set[0,0], action_plot_set[0,0]], [pos_set[0,1], action_plot_set[0,1]], [pos_set[0,2], action_plot_set[0,2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

            plots = [ax2.plot([pos_set[0,0]], [pos_set[0,1]], [pos_set[0,2]], 'o') , ax2.add_artist(a)]

            #Center
            ax2.scatter(self.origin[0], self.origin[1], self.origin[2], s=50, c='k', )

            # Axis
            ax2.plot(center_line[:,0], center_line[:,1], center_line[:,2], 'k--')

            ax2.set_xlim(-10., 10.)
            ax2.set_xlabel('X')

            ax2.set_ylim(-10., 10.)
            ax2.set_ylabel('Y')

            ax2.set_zlim(-10., 10.)
            ax2.set_zlabel('Z')

            ax2.set_title('Simulator for Rotational_model')

            # Creating the Animation object
            pos_act_ani = animation.FuncAnimation(fig2, self.update_animation, self.Data_len, fargs=(pos_set, action_plot_set, plots, ax2), interval=500, blit=False)

        plt.show()


    def update_animation(self, num, pos_set, action_plot_set, plots, ax):
        # plots[0].pop(0).remove()
        # plots[1].pop(0).remove()

        plots[0] = ax.plot([pos_set[num,0]], [pos_set[num, 1]], [pos_set[num,2]], 'bo')

        time.sleep(0.1)
        # plots[1] = ax2.plot([pos_set[num,0], action_plot_set[num,0]], [pos_set[num,1], action_plot_set[num,1]], [pos_set[num,2], action_plot_set[num,2]], 'g')
        
        plots[1].remove()
        a = Arrow3D([pos_set[num,0], action_plot_set[num,0]], [pos_set[num,1], action_plot_set[num,1]], [pos_set[num,2], action_plot_set[num,2]], mutation_scale=20, 
                    lw=3, arrowstyle="-|>", color="r")
        plots[1] = ax.add_artist(a)

        if num >= len(pos_set)-2:
            raw_input('Press Enter to close Animation')
            sys.exit(1)
        return plots


    def save_data(self, data, filename="Rotational_model.txt"):
        print "Data saved in: ", filename
        data_new = [{'origin': self.origin, 'axis': self.axis, 'radius': self.rad, 'Reference vector': self.ref_vector, 'Positions': data[0], 'Actions' : data[1]}]
        pickle.dump(data_new, open(str(filename), "wb"))



def main():
    rot_model = Rotational_model()
    rot_model.print_attributes(True)

    pos = deepcopy(rot_model.ref_vector) # start at the ref_vector
    pos_set = [pos.T]
    action_set = deepcopy(rot_model.generate_actions())
    # print "Action_set = ", action_set

    for action in action_set:
        pos = deepcopy(rot_model.simulate_one_step(pos, action))
        pos_set.append(pos.T)

    pos_set = np.array(pos_set)
    # print "Set of poses: \n ", pos_set

    do_plot = False
    do_animate = False

    rot_model.visualize([pos_set, action_set], do_plot, do_animate)

    # Save data
    filename = "../data/sim_data/Rotational_data.txt"
    rot_model.save_data([pos_set, action_set], filename)



if __name__ == '__main__':
    main()