import numpy as np
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import matplotlib.animation as animation
import time
import sys
import cPickle as pickle
import numpy.linalg as la

def py_angle(v1, v2):
    # This Always returns a value in [0, 2*pi]. This is in radians
    return np.arctan2(la.norm(np.cross(v1,v2)),np.dot(v1,v2))

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)


def plot_actions(data):
    pos_set = data[0][:,:3]
    action_set = data[1]
    # Actions
    action_plot_set = []
    for i in range(len(data[0])):
        action_plot_set.append(pos_set[i] + 0.001*action_set[i])
    action_plot_set = np.array(action_plot_set)
    fig = plt.figure(figsize=(10,10))
    ax = fig.gca(projection='3d')
    # ax.plot(pos_set[:,0], pos_set[:,1], pos_set[:,2])
    ax.scatter(pos_set[:,0], pos_set[:,1], pos_set[:,2], s=100)
    # Actions
    for i, pt in enumerate(action_plot_set):
        a = Arrow3D([pos_set[i,0], pt[0]], [pos_set[i,1], pt[1]], [pos_set[i,2], pt[2]], mutation_scale=20, lw=3, arrowstyle="-|>", color="g")
        ax.add_artist(a)
    # ax.axis('equal')
    sc = 0.1
    ax.set_xlim(-sc, sc)
    ax.set_xlabel('X')
    ax.set_ylim(-sc, sc)
    ax.set_ylabel('Y')
    ax.set_zlim(-sc, sc)
    ax.set_zlabel('Z')
    ax.set_title('Simulator for Articulation Models')
    ax.view_init(90,0)
    # ax.text(-0.12,0.0,0., 'Time Step', fontsize=15)
    plt.show()


#####################################
''' Animation '''

def update_animation(num, pos_set, action_plot_set, plots, ax):
    # plots[0].pop(0).remove()

    plots[0] = ax.plot([pos_set[num,0]], [pos_set[num, 1]], [pos_set[num,2]], 'bo', ms=15)
    
    time.sleep(0.1)
    
    # Actions
    plots[1].remove()
    a = Arrow3D([pos_set[num,0], action_plot_set[num,0]], [pos_set[num,1], action_plot_set[num,1]], [pos_set[num,2], action_plot_set[num,2]], mutation_scale=15, lw=5, arrowstyle="-|>", color="r")

    plots[1] = ax.add_artist(a)

    # TimeStep
    plots[2].remove()
    # TimeStep text
    plots[2] = ax.text(-0.25, -0.02,0., 'Time Step: ' + str(num), fontsize=30)


    if num >= len(pos_set)-2:
        raw_input('Press Enter to close Animation')
        # sys.exit(1)
    return plots


def animate_actions(data):
    pos_set = data[0][:,:3]
    action_set = data[1]

    # Actions
    action_plot_set = []
    for i in range(len(data[0])):
        action_plot_set.append(pos_set[i] + 1.0*action_set[i])
    action_plot_set = np.array(action_plot_set)
    action_plot_set *= -1.

    fig2 = plt.figure(figsize=(10,10))
    ax2 = Axes3D(fig2)

    a = Arrow3D([pos_set[0,0], action_plot_set[0,0]], [pos_set[0,1], action_plot_set[0,1]], [pos_set[0,2], action_plot_set[0,2]], mutation_scale=15, lw=5, arrowstyle="-|>", color="r")
    
    time_text = ax2.text(-0.25, -0.02,0., 'Time Step: 0', fontsize=30)

    plots = [ax2.plot([pos_set[0,0]], [pos_set[0,1]], [pos_set[0,2]], 'o') , ax2.add_artist(a), time_text]

    sc = 0.2
    # cp = np.array([0., 0., 0.])
    # ax2.plot([cp[0]-0.15, cp[0]+0.15], [cp[1],cp[1]], [cp[2],cp[2]],'g--', linewidth=10)

    ax2.set_xlim(-sc, sc)
    ax2.set_xlabel('X')
    ax2.set_ylim(-sc, sc)
    ax2.set_ylabel('Y')
    ax2.set_zlim(-sc, sc)
    ax2.set_zlabel('Z')

    plt.rc('xtick', labelsize=25) 
    plt.rc('ytick', labelsize=25)
    plt.rcParams.update({'font.size': 25})

    ax2.set_title('Simulator for Articulation Models')
    # ax2.view_init(90,0)

    # Creating the Animation object
    pos_act_ani = animation.FuncAnimation(fig2, update_animation, len(data[0]), fargs=(pos_set, action_plot_set, plots, ax2), interval=200, blit=False)
    # pos_act_ani.save(str(fig_save_name) + '.gif', dpi=80, writer='imagemagick')
    plt.show()


def angular_diff_bw_pose_and_action(data):
    pos_set = data[0][:,:3]
    action_set = data[1]
    # action_set *= -1.

    delta_pose = []
    signed_norm = []
    ref_vector = pos_set[1] - pos_set[0]

    for i in range(1, len(data[0])):
        delta_pose.append(pos_set[i] - pos_set[i-1])

        if(py_angle(ref_vector, pos_set[i]-pos_set[i-1]) <= np.pi/2):
            signed_norm.append(-la.norm(pos_set[i] - pos_set[i-1]))
        else:
            signed_norm.append(la.norm(pos_set[i] - pos_set[i-1]))

    delta_pose = np.array(delta_pose)
    signed_norm = np.array(signed_norm)

    dot_set = []
    for i in range(len(delta_pose)):
        dot_set.append(np.dot(ref_vector, action_set[i]))

    dot_set = np.array(dot_set)

    ts = range(len(dot_set))
    # norm_delta_pose = la.norm(delta_pose, axis=1)
    # norm_delta_pose = signed_norm
    # norm_action = la.norm(action_set, axis=1)

    # diff_pose_action = np.array([la.norm(delta_pose[i] - dot_set[i]*(delta_pose[i]/la.norm(delta_pose[i]))) for i in range(len(dot_set))])

    # fig = plt.figure(1)
    # l1, = plt.plot(ts, ang_set, 'bx-')
    # # plt.plot(ts, [np.pi/2]*len(ts), 'r')
    # plt.plot(ts, [0.]*len(ts), 'r')
    # l2, = plt.plot(ts, (1./max(norm_delta_pose))*norm_delta_pose, 'kx-')
    # # plt.plot(ts, ang_set_2, 'r')
    # plt.title('Comparison of angle between action and pose diff')
    # plt.legend([l1, l2], ['Cos(Ang)', 'Delta pose'])
    # plt.grid('on')

    fig = plt.figure(4)
    signed_norm = (1./(max(signed_norm) - min(signed_norm)))*signed_norm
    dot_set = (1./(max(dot_set) -min(dot_set)))*dot_set
    plt.plot(ts, [0.]*len(ts), 'r')
    l1, = plt.plot(ts, signed_norm, 'kx-')
    l2, = plt.plot(ts, dot_set, 'bx-')
    plt.legend([l1, l2], ['Delta Pose', 'Dot product of Action'])
    plt.grid('on')

    # fig2 = plt.figure(2)
    # l1, = plt.plot(ts, norm_delta_pose)
    # l2, = plt.plot(ts, diff_pose_action, 'rx')
    # l3, = plt.plot(ts, dot_set, 'k')
    # # l4, = plt.plot(ts, norm_action[:-1], 'g')
    # plt.legend([l1, l2, l3], ['Delta pose', 'diff_pose_action', 'dot_set'])
    # # plt.legend([l1, l2, l3, l4], ['Delta pose', 'diff_pose_action', 'dot_set', 'Action'])
    # plt.title('Norms for comparison')
    
    # fig3 = plt.figure(3)
    # ax = fig3.add_subplot(111, projection='3d')
    # ax.scatter(delta_pose[:,0], delta_pose[:,1], delta_pose[:,2])

    plt.show()


if __name__=="__main__":
    # data = pickle.load(open('pkl_files/microwave_data.pkl','r'))
    data = pickle.load(open('../data/pkl_files/' + sys.argv[1] + '.pkl','r'))

    # plot_actions(data)
    # animate_actions(data)
    angular_diff_bw_pose_and_action(data)



