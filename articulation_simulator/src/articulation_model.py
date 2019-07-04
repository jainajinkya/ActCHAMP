#! usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler

import numpy as np
import sys, time
from copy import deepcopy
from utils import *
import rotational_model
import prismatic_model
import freebody_model
import rigid_model

#Utils
def pt_to_pose(pt):
    # Adding Generic orientation in Quaternion format
    return np.array([pt[0], pt[1], pt[2], 0., 0., 0., 1.])


def pose_to_pt(pos):
    return np.array(pos[:3])        


def pt_array_to_pose_array(pt_array):
    pose_arr = []
    for pt in pt_array:
        pose_arr.append(pt_to_pose(pt))

    return pose_arr


def get_noisy_observation(pos, cov_multiplier=1e-9):
    pos = np.array(pos)
    n_dim = np.size(pos)
    mu = np.zeros(n_dim)
    cov = cov_multiplier*np.eye(n_dim)
    err = np.random.multivariate_normal(mu, cov)
    return pos + err

def get_noisy_observations_array(pos_array):
    obs_array = []
    for pos in pos_array:
        obs_array.append(get_noisy_observation(pos))

    return obs_array



class Articulation_model(object):
    """docstring for Articulation_model"""
    def __init__(self):
        # super(Articulation_model, self).__init__()
        self.rotation = rotational_model.Rotational_model()
        self.prismatic = prismatic_model.Prismatic_model()
        self.freebody = freebody_model.Freebody_model()
        self.rigid = rigid_model.Rigid_model()

        # Global Parameters
        self.origin = np.array([0., 0., 0.])
        self.axis = np.array([0., 0., 1.])
        self.Int_res = 100
        self.Data_len = 10
        self.ori_in_quat = True


    def set_attributes(self, model_type, attributes):
        if model_type == 'rotational':
            self.rotation.origin = attributes['origin']
            self.rotation.rad = attributes['radius']
            self.rotation.axis = attributes['axis']
            self.rotation.axis /= la.norm(self.rotation.axis)
            self.rotation.ref_vector = deepcopy(self.rotation.origin + self.rotation.rad*get_perpendicular_vector(self.rotation.axis))

        elif model_type == 'prismatic':
            self.prismatic.origin = attributes['origin']
            self.prismatic.axis = attributes['axis']
            self.prismatic.axis /= la.norm(self.prismatic.axis)
            self.prismatic.origin_orientation = attributes['origin_orientation']


        elif model_type == 'rigid':
            self.rigid.origin = attributes['origin']
            self.rigid.origin_orientation = attributes['origin_orientation']
            

        elif model_type == 'freebody':
            self.freebody.origin = attributes['origin']
            self.freebody.origin_orientation = attributes['origin_orientation']
            


    def get_attributes(self, model_type):
        attributes = {}
        if model_type == 'rotational':
            attributes['origin'] = self.rotation.origin 
            attributes['radius'] = self.rotation.rad 
            attributes['axis'] = self.rotation.axis 

        elif model_type == 'prismatic':
            attributes['origin'] = self.prismatic.origin  
            attributes['axis'] = self.prismatic.axis  

        elif model_type == 'rigid':
            attributes['origin'] = self.rigid.origin

        elif model_type == 'freebody':
            attributes['origin'] = self.freebody.origin

        return attributes


    def get_attributes_verbose(self, model_type):
        attributes = {}
        if model_type == 'rotational':
            attributes['rot_center.x'] = self.rotation.origin[0]
            attributes['rot_center.y'] = self.rotation.origin[1] 
            attributes['rot_center.z'] = self.rotation.origin[2]
            attributes['rot_radius'] = self.rotation.rad 

            if self.ori_in_quat:
                q = quaternion_from_euler(self.rotation.axis[0], self.rotation.axis[1], self.rotation.axis[2])
                q2 = quaternion_from_euler(self.rotation.ref_vector[0], self.rotation.ref_vector[1], self.rotation.ref_vector[2])
                 
                ## In Quaternion Format
                attributes['rot_axis.x'] = q[0]
                attributes['rot_axis.y'] = q[1]
                attributes['rot_axis.z'] = q[2]
                attributes['rot_axis.w'] = q[2]

                attributes['rot_orientation.x'] = q2[0]
                attributes['rot_orientation.y'] = q2[1]
                attributes['rot_orientation.z'] = q2[2]
                attributes['rot_orientation.w'] = q2[2]

            else:
                ## In Euler Format
                attributes['rot_axis.x'] = self.rotation.axis[0]
                attributes['rot_axis.y'] = self.rotation.axis[1]
                attributes['rot_axis.z'] = self.rotation.axis[2]

                attributes['rot_orientation.x'] = self.rotation.ref_vector[0]
                attributes['rot_orientation.y'] = self.rotation.ref_vector[1]
                attributes['rot_orientation.z'] = self.rotation.ref_vector[2]


        elif model_type == 'prismatic':
            attributes['prismatic_dir.x'] = self.prismatic.axis[0]  
            attributes['prismatic_dir.y'] = self.prismatic.axis[1]  
            attributes['prismatic_dir.z'] = self.prismatic.axis[2]  
            attributes['rigid_position.x'] = self.prismatic.origin[0]  
            attributes['rigid_position.y'] = self.prismatic.origin[1]  
            attributes['rigid_position.z'] = self.prismatic.origin[2]

            if self.ori_in_quat:
                q = quaternion_from_euler(self.prismatic.origin_orientation[0], self.prismatic.origin_orientation[1], self.prismatic.origin_orientation[2])
                 
                ## In Quaternion Format
                attributes['rigid_orientation.x'] = q[0]
                attributes['rigid_orientation.y'] = q[1]
                attributes['rigid_orientation.z'] = q[2]
                attributes['rigid_orientation.w'] = q[3]

            else:
                ## In Euler Format
                attributes['rigid_orientation.x'] = self.rigidation.ref_vector[0]
                attributes['rigid_orientation.y'] = self.rigidation.ref_vector[1]
                attributes['rigid_orientation.z'] = self.rigidation.ref_vector[2]


        elif model_type == 'rigid':
            attributes['rigid_position.x'] = self.rigid.origin[0]
            attributes['rigid_position.y'] = self.rigid.origin[1] 
            attributes['rigid_position.z'] = self.rigid.origin[2]

            if self.ori_in_quat:
                q = quaternion_from_euler(self.rigid.origin_orientation[0], self.rigid.origin_orientation[1], self.rigid.origin_orientation[2])
                 
                ## In Quaternion Format
                attributes['rigid_orientation.x'] = q[0]
                attributes['rigid_orientation.y'] = q[1]
                attributes['rigid_orientation.z'] = q[2]
                attributes['rigid_orientation.w'] = q[3]

            else:
                ## In Euler Format
                attributes['rigid_orientation.x'] = self.rigid.origin_orientation[0]
                attributes['rigid_orientation.y'] = self.rigid.origin_orientation[1]
                attributes['rigid_orientation.z'] = self.rigid.origin_orientation[2]

        elif model_type == 'freebody':
            attributes['freebody_origin.x'] = self.freebody.origin[0]
            attributes['freebody_origin.y'] = self.freebody.origin[1] 
            attributes['freebody_origin.z'] = self.freebody.origin[2]

            if self.ori_in_quat:
                q = quaternion_from_euler(self.freebody.origin_orientation[0], self.freebody.origin_orientation[1], self.freebody.origin_orientation[2])
                 
                ## In Quaternion Format
                attributes['freebody_orientation.x'] = q[0]
                attributes['freebody_orientation.y'] = q[1]
                attributes['freebody_orientation.z'] = q[2]
                attributes['freebody_orientation.w'] = q[3]

            else:
                ## In Euler Format
                attributes['freebody_orientation.x'] = self.freebody.origin_orientation[0]
                attributes['freebody_orientation.y'] = self.freebody.origin_orientation[1]
                attributes['freebody_orientation.z'] = self.freebody.origin_orientation[2]

        return attributes

    def print_attributes(self, arg=True):
        if model_type == 'rotational':
            print "Rotation Model Attributes:"
            print "Axis : ", self.rotation.axis
            print "origin : ", self.rotation.origin
            print "Radius : ", self.rotation.rad
            print "Reference Vector : ", self.rotation.ref_vector


        elif model_type == 'prismatic':
            print "Prismatic Model Attributes:"
            print "Axis : ", self.prismatic.axis
            print "origin : ", self.prismatic.origin


        elif model_type == 'rigid':
            print "Rigid Model Attributes:"
            print "origin : ", self.rigid.origin

        elif model_type == 'freebody':
            print "Freebody Model Attributes:"
            print "origin : ", self.freebody.origin

    
    def generate_actions(self, scale=0.01):
        arr = np.zeros((self.Data_len, 3))

        # arr = np.random.uniform(-0.002*scale, 0.002*scale, (self.Data_len, 3))
        arr[:,1] = scale

        # p = np.random.randint(self.Data_len, size=self.Data_len)     
        # for i in range(self.Data_len):
        #     if p[i] < 0.05*self.Data_len:
        #         arr[i,1] = 0.01*scale
        # return arr

        # for i, theta in enumerate(np.linspace(0, 0.1, num=self.Data_len)):
        #     arr[i, 0] = scale*np.cos(theta)
        #     arr[i, 1] = scale*np.sin(theta)

        return arr


    def check_active_model(self, pos):
        ## This function decides how different models are activated

        ## Assuming it's a revolute model if theta > 30 deg or else it is a rigid model
        # r_vector = pos - self.origin
        # theta = py_angle(r_vector, self.rotation.ref_vector)
        # # print "Theta: ", theta
        # if theta < 0.35 : ##0.35 corresponds to 20 degrees
        #     return 'rigid'
        # else:
        #     return 'rotational'

        ## Assuming it's a prismatic model if its y < 2.0 or else it is a rigid model
        # if((pos - self.prismatic.origin)[1] < 0.0):
        #     return 'prismatic'
        # else:
        #     return 'rigid'

        if((pos - self.prismatic.origin)[1] > 0.0):
            return 'prismatic'
        else:
            return 'rotational'


    def simulate_one_step(self, pos, action):
        # Check the active model
        model_type = self.check_active_model(pos)

        if model_type == 'rotational':
            new_pos = self.rotation.simulate_one_step(pos, action)

        elif model_type == 'prismatic':
            new_pos = self.prismatic.simulate_one_step(pos, action)

        elif model_type == 'rigid':
            # dummy = self.rigid.simulate_one_step(pos, action)
            # new_pos = get_noisy_observation(dummy, 1e-5)
            new_pos = self.rigid.simulate_one_step(pos, action)


        elif model_type == 'freebody':
            new_pos = self.freebody.simulate_one_step(pos, action)

        return new_pos


    def visualize(self, data, do_plot=False, do_animate=False, fig_save_name='1'):
        pos_set = data[0]
        action_set = data[1]

        # Calculations for plotting
        # Axis
        center_line = []
        for i in np.arange(-30., 30., 0.5):
            center_line.append((self.origin + i*self.prismatic.axis).T)
        
        center_line = np.array(center_line)

        # Actions
        action_plot_set = []
        for i in range(self.Data_len):
            action_plot_set.append(pos_set[i] + 3.0*action_set[i])

        action_plot_set = np.array(action_plot_set)


        if do_plot:
            fig = plt.figure(figsize=(25,25))
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
            sc = 0.5
            ax.set_xlim(-sc, sc)
            ax.set_xlabel('X')
            ax.set_ylim(-sc, sc)
            ax.set_ylabel('Y')
            ax.set_zlim(-sc, sc)
            ax.set_zlabel('Z')

            ax.set_title('Simulator for Articulation Models')
            ax.view_init(90,0)
            ax.text(-0.12,0.0,0., 'Time Step', fontsize=15)


        # Animation

        if do_animate:
            fig2 = plt.figure(figsize=(25,25))
            ax2 = Axes3D(fig2)
            dt_scale = 1.
            pos_set *= dt_scale
            # action_plot_set *= 1.2*dt_scale

            a = Arrow3D([pos_set[0,0], action_plot_set[0,0]], [pos_set[0,1], action_plot_set[0,1]], [pos_set[0,2], action_plot_set[0,2]], mutation_scale=40, lw=15, arrowstyle="-|>", color="r")

            # TimeStep text
            time_text = ax2.text(-0.25, -0.02,0., 'Time Step: 0', fontsize=30)

            plots = [ax2.plot([pos_set[0,0]], [pos_set[0,1]], [pos_set[0,2]], 'o') , ax2.add_artist(a), time_text]

            #Center
            ax2.scatter(self.origin[0], self.origin[1], self.origin[2], s=200, c='k', )

            # Axis
            ax2.plot(center_line[:,0], center_line[:,1], center_line[:,2], 'k--')

            # Changepoint
            # Rot_mat = rotation_matrix(self.axis, -0.35)
            # cp = self.origin + np.dot(Rot_mat, 1.2*self.rotation.ref_vector)
            # ax2.plot([self.origin[0], cp[0]], [self.origin[1],cp[1]], [self.origin[2],cp[2]],'g--', linewidth=10)

            sc = 0.2
            cp = np.array([0., 0., 0.])
            ax2.plot([cp[0]-0.15, cp[0]+0.15], [cp[1],cp[1]], [cp[2],cp[2]],'g--', linewidth=10)

            ax2.set_xlim(-sc, sc)
            ax2.set_xlabel('X')
            ax2.set_ylim(-sc, sc)
            ax2.set_ylabel('Y')
            ax2.set_zlim(-sc, sc)
            ax2.set_zlabel('Z')

            # plt.rcParams['savefig.bbox'] = 'tight'
            plt.rc('xtick', labelsize=25) 
            plt.rc('ytick', labelsize=25)
            plt.rcParams.update({'font.size': 25})

            ax2.set_title('Simulator for Articulation Models')
            ax2.view_init(90,0)

            # Creating the Animation object
            pos_act_ani = animation.FuncAnimation(fig2, self.update_animation, self.Data_len, fargs=(pos_set, action_plot_set, plots, ax2), interval=200, blit=False)
            pos_act_ani.save(str(fig_save_name) + '.gif', dpi=80, writer='imagemagick')

        # plt.show()


    def update_animation(self, num, pos_set, action_plot_set, plots, ax):
        # plots[0].pop(0).remove()

        plots[0] = ax.plot([pos_set[num,0]], [pos_set[num, 1]], [pos_set[num,2]], 'bo', ms=15)
        
        time.sleep(0.1)
        
        # Actions
        plots[1].remove()
        a = Arrow3D([pos_set[num,0], action_plot_set[num,0]], [pos_set[num,1], action_plot_set[num,1]], [pos_set[num,2], action_plot_set[num,2]], mutation_scale=40, lw=15, arrowstyle="-|>", color="r")

        plots[1] = ax.add_artist(a)

        # TimeStep
        plots[2].remove()
        # TimeStep text
        plots[2] = ax.text(-0.25, -0.02,0., 'Time Step: ' + str(num), fontsize=30)


        if num >= len(pos_set)-2:
            raw_input('Press Enter to close Animation')
            # sys.exit(1)
        return plots


    def save_data(self, data, filename="Articulation_data.txt"):
        print "Data saved in: ", filename
        # data_new = [{'origin': self.origin, 'axis': self.axis, 'Positions': data[0], 'Actions' : data[1]}]
        # pickle.dump(data_new, open(str(filename), "wb"))
        pickle.dump(data, open(str(filename), "wb"))



    def generate_groundTruth_datafile(self, pos_set, f_name_string):
        all_models = []
        old_model_type = deepcopy(self.check_active_model(pos_set[0]))
        params_0 = deepcopy(self.get_attributes_verbose(old_model_type))
        data_0 = deepcopy({'params': params_0, 'log_likelihood': 0., 'model_evidence': 0.})
        model_0 = deepcopy({'Model': old_model_type, 'Start': 0, 'End': 0, 'Length':0, 'Data': data_0})
        all_models.append(model_0)
        n_models = 0

        for i, pos in enumerate(pos_set):
            # Check which model does it belong to. 
            model_type = self.check_active_model(pos)
            if model_type == old_model_type:
                all_models[n_models]['End'] = i
                all_models[n_models]['Length'] = i - all_models[n_models]['Start'] + 1
                continue
            else:
                # If change in model, add a new data member in the dictionary of models
                # Calculate the length for which this model was active
                old_model_type = deepcopy(model_type)
                params_0 = deepcopy(self.get_attributes_verbose(old_model_type))
                data_0 = deepcopy({'params': params_0, 'log_likelihood': 0., 'model_evidence': 0.})
                model_0 = deepcopy({'Model': old_model_type, 'Start': i, 'End': i, 'Length':0, 'Data': data_0})
                all_models.append(model_0)
                n_models += 1

        
        f_name = "../data/simulations/sim_data/" + f_name_string + "_groundTruth.txt"
        gt_file = open(f_name, 'w')
        pickle.dump(all_models, gt_file)
        gt_file.close()
    


def main():
    articulate = Articulation_model()

    # ## Setting up rotational model parameters
    attri = {'origin': articulate.origin, 'axis': articulate.axis, 'radius': 0.1}
    articulate.set_attributes('rotational', attri)
    # Rot_mat = rotation_matrix(articulate.axis, -0.43)
    Rot_mat = rotation_matrix(articulate.axis, -1.0)
    pos = articulate.origin + np.dot(Rot_mat, articulate.rotation.ref_vector)

    # ## Setting up Prismatic model parameters
    attri = {'origin': np.array([0., 0., 0.]), 'origin_orientation': np.array([0., 0., 0.]), 'axis': np.array([0., 1., 0.])}
    articulate.set_attributes('prismatic', attri)
    articulate.axis = np.array([0., 1., 0.])
    # pos = np.array([0., -0.05, 0.])


    # Setting up rigid model parameters
    # attri.clear()
    # attri = {'origin': np.array([0., 0.0, 0.]), 'origin_orientation': np.array([0., 0., 0.])}
    # articulate.set_attributes('rigid', attri)

    print "Initial pose: ", pos
    pos_set = [pos.T]
    action_set = deepcopy(articulate.generate_actions())

    for action in action_set:
        pos = deepcopy(articulate.simulate_one_step(pos, action))
        pos_set.append(pos.T)

    pos_set = np.array(pos_set)

    # Save data for champ
    obs_array = get_noisy_observations_array(pos_set)
    new_pos_set = pt_array_to_pose_array(obs_array)

    filename = "../data/simulations/sim_data/" + str(sys.argv[1]) + ".txt"
    articulate.save_data([new_pos_set, action_set], filename)

    do_plot = False
    do_animate = True

    articulate.visualize([pos_set, action_set], do_plot, do_animate, str(sys.argv[1]))

    # Generate Data file for test suite
    # Setting up rigid model parameters
    attri.clear()
    # attri = {'origin': pos_set[-1], 'origin_orientation': np.array([0., 0., 0.])}
    # articulate.set_attributes('rigid', attri)

    articulate.generate_groundTruth_datafile(pos_set, str(sys.argv[1]))


if __name__ == "__main__":
    main()
