#!/usr/bin/env python

""""
Ref: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/LQRRRTStar/lqr_rrt_star.py
"""

import numpy as np
import scipy.linalg as la
import copy
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import cPickle as pkl
import bz2
import os.path

class LQR():
    def solve_DARE(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        X = Q
        maxiter = 150
        eps = 0.01

        for i in range(maxiter):
            Xn = A.T.dot(X).dot(A) - A.T.dot(X.dot(B)).dot( \
                la.inv(R + B.T.dot(X.dot(B)))).dot(B.T.dot(X.dot(A))) + Q
            if (abs(Xn - X)).max() < eps:
                break
            X = Xn

        return Xn


    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T.dot(X.dot(B)) + R).dot(B.T.dot(X.dot(A)))

        eigVals, eigVecs = la.eig(A - B.dot(K))

        return K, X, eigVals



class Node():
    """
    RRT Node
    """
    def __init__(self, coords, cov=np.eye(4)):
        self.coords = coords ## numpy nd array
        self.cov = cov
        self.path_x = []
        self.path_s = []
        self.path_u = []
        self.cost = 0.0
        self.parent = None



class Dynamics(object):
    def __init__(self, nState, nInput):
        self.nState = nState
        self.nInput = nInput
        self.A = np.eye(nState)
        self.B = np.eye(nState)
        self.Q = 0.5*np.eye(nState)
        self.R = 0.5*np.eye(nInput)
        self.LQR = LQR()

    def A_mat(self, x, u=None):
        return self.A

    def B_mat(self, x, u=None):
        return self.B

    def Q_mat(self):
        return self.Q

    def R_mat(self):
        return self.R

    def LQR_mats(self, x, u=None):
        A = self.A_mat(x, u)
        B = self.B_mat(x, u)
        Q = self.Q_mat()
        R = self.R_mat()

        return self.LQR.dlqr(A, B, Q, R)

    def LQR_control(self, x):
        Kopt, S, ev = self.LQR_mats(x)
        return -1.*Kopt.dot(x)

    def propagateState(self, x, u):
        return self.A_mat(x, u).dot(x) + self.B_mat(x, u).dot(u)


class RRT(object):

    """
    Class for RRT Planning
    """
    DYNAMICS_INTEGRATION_STEPS = 1
    GOAL_THRESHOLD = 2.0
    COV_IDS = np.triu_indices(4)
    FIG = None
    # FILE_NAME = "rrt_tree_1.pbz2"
    # FILE_NAME = os.path.dirname(__file__) + "/rrt_tree_1.pkl"
    FILE_NAME = os.path.dirname(__file__) + "/rrt_tree_1.pbz2"


    # def __init__(self, start, goal, obstacle_list, sample_domain, dynamics, goal_sample_rate=0.1, n_samples=200):
    def __init__(self, start, goal, obstacle_list, sample_domain, dynamics, start_cov=np.eye(4), goal_cov=1e-2*np.eye(4), goal_sample_rate=0.1, n_samples=200):

        self.start = Node(start, start_cov)
        self.goal = Node(goal, goal_cov)
        self.sample_domain = sample_domain
        self.goal_sample_rate = goal_sample_rate
        self.n_samples = n_samples
        self.dynamics = dynamics
        self.obstacle_list = obstacle_list

    def planPath(self, animation=False, print_tree=False, use_prebuilt_tree=False):
        if use_prebuilt_tree:
            self.loadTree(self.FILE_NAME)
        else:
            self.tree = [self.start]

        for i in range(self.n_samples):
            x_rand = self.getSample()
            x_nearest_id = self.LQRNearest(x_rand)
            x_new = self.LQRSteer(x_nearest_id, x_rand)

            if self.collisonFree([x_new]): ## Check if the traj is collision free
                X_near_inds = self.LQRNear(x_new) ## Set of nearby nodes
                x_new = self.chooseParent(X_near_inds, x_new)
                self.tree.append(x_new)
                self.rewire(X_near_inds, x_new)

            if animation:
                self.drawGraph(x_new)

            if np.linalg.norm(x_new.coords- self.goal.coords) < self.GOAL_THRESHOLD:
                break

        ## Generate Final course
        if print_tree:
            self.printTree()

        last_index = self.getBestLastIndex()

        if last_index is None:
            return None, None, None

        path, s_path, u_path = self.generateFinalPath(last_index)
    
        if len(path) == 0:
            print "No path Found"
        else:
            print "Start Pt: ", self.start.coords.T
            print "Goal Pt: ", self.goal.coords.T
            print "Final Pt Reached: ", path[-1].T
            print "Final Covariance: \n", np.round(self.tree[last_index].cov, 3)
            print "Path Length: ", len(path)
            # print "\nFinal Path: ", path
            # print "\nFinal Cov Path: ", s_path
            # print "\nFinal u Path: ", u_path


        if animation:
            self.plotPath(path)

        ## Save tree for future use
        self.saveTree(self.FILE_NAME)

        return path, s_path, u_path


    def getSample(self):
        if np.random.random_sample() < self.goal_sample_rate:
            rnd = self.goal.coords
        else:
            rnd = np.array([np.random.uniform(self.sample_domain[i][0], self.sample_domain[i][1]) for i in range(len(self.start.coords))])

        return Node(rnd)


    def LQRNearest(self, x_rand):
        K, S, eig = self.dynamics.LQR_mats(x_rand.coords.T)

        min_cost = np.inf
        x_nearest_tree_id = 0
        for i, v in enumerate(self.tree):
            cost = (v.coords - x_rand.coords).T.dot(S).dot(v.coords - x_rand.coords)

            if cost <= min_cost:
                min_cost = cost
                x_nearest_tree_id = i

        return x_nearest_tree_id


    def LQRSteer(self, x_nearest_id, x_end):
        x_start = self.tree[x_nearest_id]

        X = (x_start.coords - x_end.coords).T
        u = self.dynamics.LQR_control(X)

        Q = self.dynamics.Q_mat()
        R = self.dynamics.R_mat()
        
        x = copy.copy(x_start.coords)
        cov = copy.copy(x_start.cov)
        path_x = [x]
        path_u = [u]
        path_s = [cov[self.COV_IDS]]
        cost = 0

        for i in range(self.DYNAMICS_INTEGRATION_STEPS):
            x_new,cov_new = self.dynamics.propagateState(x, u/self.DYNAMICS_INTEGRATION_STEPS, cov=cov)

            cost += (x_new-x).T.dot(Q.dot(x_new-x)) + u.T.dot(R.dot(u))        
            x = copy.copy(x_new)
            cov = copy.copy(cov_new)

        path_x.append(copy.copy(x))
        path_s.append(copy.copy(cov[self.COV_IDS]))

        new_node = copy.deepcopy(x_start)
        new_node.coords = x
        new_node.cov = cov
        new_node.path_x = path_x
        new_node.path_s = path_s
        new_node.path_u = path_u
        new_node.cost += cost
        new_node.parent = x_nearest_id

        # print "x_new in LQRSteer: ", x_new.T
        return new_node


    def LQRNear(self, x_new):
        K, S, eig = self.dynamics.LQR_mats(x_new.coords)

        nnode = len(self.tree)
        r = 200.0*((math.log(nnode) / nnode)**(1/float(self.dynamics.nState)))
        dlist = [(v.coords-x_new.coords).dot(S.dot(v.coords-x_new.coords).T) for v in self.tree]

        nearInds = [dlist.index(i) for i in dlist if i <= r ]

        return nearInds

    def chooseParent(self, X_near, x_new):
        if not X_near:
            return x_new

        min_cost = np.inf
        min_id = None

        for i in X_near:
            t_node = self.LQRSteer(i, x_new)

            cur_cost = t_node.cost + self.tree[i].cost
            if cur_cost <= min_cost:
                min_cost = cur_cost
                min_id = i

        if min_id is None:
            return x_new
        else:
            new_node = self.LQRSteer(min_id, x_new)
            return new_node


    def collisonFree(self, traj):
        return True


    def rewire(self, X_near, x_new):
        nnode = len(self.tree)
        # print "\n>>> Number of neighbours considered for rewiriing: ", len(X_near)

        for i in X_near:
            near_node = self.tree[i]
            t_node = self.LQRSteer(nnode-1, near_node)

            obstacle_ok = self.collisonFree([t_node])
            improve_cost = near_node.cost > t_node.cost

            if obstacle_ok and improve_cost:
                self.tree[i] = t_node


    def getBestLastIndex(self, goal_strict=False):
        if goal_strict:
            goal_inds = []
            for i, node in enumerate(self.tree):
                if np.linalg.norm(self.goal.coords - node.coords) < self.GOAL_THRESHOLD:
                    goal_inds.append(i)

            if len(goal_inds) < 1:
                return None
            else:
                return np.argmin([self.tree[i].cost for i in goal_inds])

        # min_cost = min([self.tree[i].cost for i in goal_inds])

        # for i in goal_inds:
        #     if self.tree[i].cost == min_cost:
        #         return i

        else:
            print "Estimating the current best path."
            min_cost = np.inf
            best_id = None
            for i, node in enumerate(self.tree):
                cur_cost = np.linalg.norm(self.goal.coords - node.coords)
                if cur_cost <= min_cost:
                    best_id = i
                    min_cost = cur_cost

            print "Final Cost: ", min_cost
            return best_id 


    def generateFinalPath(self, goal_ind):
        # final_path = [self.goal.coords]
        final_path = []
        s_path = []
        u_path = []

        while self.tree[goal_ind].parent is not None:
            node = self.tree[goal_ind]
            # print "Node path_s: ", node.path_s
            # print "node path_u: ", node.path_u

            for i, coords in enumerate(reversed(node.path_x)):
                if i > 0:
                    final_path.append(coords)

            for i, s in enumerate(reversed(node.path_s)):
                if i > 0:
                    s_path.append(s)
            
            u_path.append(node.path_u[0])

            goal_ind = node.parent

        return list(reversed(final_path)), list(reversed(s_path)), list(reversed(u_path[1:]))


    def printTree(self):
        for i, v in enumerate(self.tree):
            print "Node: ", i, "\tCoords: ", v.coords.T, "\tParent: ", v.parent, "\n" 

    def plotPath(self, path):
        if np.shape(path)[1] == 2:
            path_x = [pt[0] for pt in path]
            path_y = [pt[1] for pt in path]
            plt.plot(path_x, path_y, 'k')
            plt.plot(self.start.coords[0], self.start.coords[1], "ok")
            plt.plot(self.goal.coords[0], self.goal.coords[1], "og")
            plt.show()
        else:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            path_x = []
            path_y = []
            path_z = []
            path_theta = []
            for pt in path:
                path_x.append(pt[0])
                path_y.append(pt[1])
                path_z.append(pt[2])
                path_theta.append(pt[3])


            ax.plot(path_x, path_y, path_z, c="b")
            ax.scatter(path_x, path_y, path_z, c="b", marker="x", s=100)

            for (ox, oy, oz, size) in self.obstacle_list:
                ax.plot(ox, oy, oz, c="k", ms=30 * size)

            ax.scatter(path_x[0], path_y[0], path_z[0], c="k", marker="s", s=150)
            ax.scatter(path_x[-1], path_y[-1], path_z[-1], c="g", marker="s", s=150)

            ax.set_xlim(self.sample_domain[0][0], self.sample_domain[0][1])
            ax.set_ylim(self.sample_domain[1][0], self.sample_domain[1][1])
            ax.set_zlim(self.sample_domain[2][0], self.sample_domain[2][1])

            ax.set_xlabel("X_axis")
            ax.set_ylabel("Y_axis")
            ax.set_zlabel("Z_axis")

            fig2 = plt.figure(2)
            t_steps = range(len(path_theta))
            plt.plot(t_steps, path_theta)
            plt.scatter(t_steps, path_theta, c='b', marker='x', s=20)
            plt.scatter(t_steps[0], path_theta[0], c='k', marker='s', s=50)
            plt.scatter(t_steps[-1], path_theta[-1], c='g', marker='s', s=50)

            plt.xlabel("Time Steps")
            plt.ylabel(r"$\mathbf{\theta}$")
            plt.ylim([self.sample_domain[3][0], self.sample_domain[3][1]])

            plt.show()


    def drawGraph(self, rnd=None):
        if len(rnd.coords) ==2:
            plt.clf()
            if rnd is not None:
                plt.plot(rnd.coords[0], rnd.coords[1], "^k")

            for node in self.tree:
                if node.parent is not None:
                    path_x = [pt[0] for pt in node.path_x]
                    path_y = [pt[1] for pt in node.path_x]
                    plt.plot(path_x, path_y, "-g")

            for (ox, oy, size) in self.obstacle_list:
                plt.plot(ox, oy, "ok", ms=30 * size)

            plt.plot(self.start.coords[0], self.start.coords[1], "sk")
            plt.plot(self.goal.coords[0], self.goal.coords[1], "sg")

            plt.axis([-2, 15, -2, 15])
            plt.grid(True)
            plt.pause(0.01)
        
        else:
            if self.FIG is None:
                self.FIG = plt.figure()
                self.ax = self.FIG.add_subplot(111, projection='3d')

            self.ax.cla()
            if rnd is not None:
                self.ax.scatter(rnd.coords[0], rnd.coords[1], rnd.coords[2], c="k", marker="^")

            for node in self.tree:
                if node.parent is not None:
                    path_x = [pt[0] for pt in node.path_x]
                    path_y = [pt[1] for pt in node.path_x]
                    path_z = [pt[2] for pt in node.path_x]
                    self.ax.plot(path_x, path_y, path_z, c="b")

            for (ox, oy, oz, size) in self.obstacle_list:
                self.ax.plot(ox, oy, oz, c="k", ms=30 * size)

            self.ax.scatter(self.start.coords[0], self.start.coords[1], self.start.coords[2], c="k", marker="s", s=150)
            self.ax.scatter(self.goal.coords[0], self.goal.coords[1], self.goal.coords[2], c="g", marker="s", s=150)

            self.ax.set_xlim(self.sample_domain[0][0], self.sample_domain[0][1])
            self.ax.set_ylim(self.sample_domain[1][0], self.sample_domain[1][1])
            self.ax.set_zlim(self.sample_domain[2][0], self.sample_domain[2][1])
            # self.ax.set_zlim(self.sample_domain[0][0], self.sample_domain[0][1])

            self.ax.set_xlabel("X_axis")
            self.ax.set_ylabel("Y_axis")
            self.ax.set_zlabel("Z_axis")
            
            plt.pause(0.01)


    def loadTree(self, file_name):
        if os.path.exists(file_name):
            # tree_file = open(file_name, "rb")
            # self.tree = pkl.load(tree_file)
            # tree_file.close()

            with bz2.BZ2File(file_name, 'r') as f:
                self.tree = pkl.load(f)
            
            print ">>>>>>>>>  Loaded tree with ", len(self.tree), " samples <<<<<<<<<"
            ## Appending current start with the loaded tree
            x_nearest_id = self.LQRNearest(self.start)
            x_new = self.LQRSteer(x_nearest_id, self.start)

            if self.collisonFree([x_new]): ## Check if the traj is collision free
                X_near_inds = self.LQRNear(x_new) ## Set of nearby nodes
                x_new = self.chooseParent(X_near_inds, x_new)
                self.tree.append(x_new)
                self.rewire(X_near_inds, x_new)
        else:
            open(file_name, 'ab').close()
            self.tree = [self.start]

    def saveTree(self, file_name):
        # tree_file = bz2.BZ2File(self.FILE_NAME, mode='w')
        with bz2.BZ2File(file_name, 'w') as f:
            pkl.dump(self.tree, f)
        # tree_file = open(file_name, 'w')
        # pkl.dump(self.tree, tree_file)
        # tree_file.close()



def main():
    start = np.array([0., 0.])
    goal = np.array([6.0, 7.0])

    obstacle_list = []
    sample_domain = [[-2., 15.], [-2., 15.]]
    n_samples = 1000

    dynamics = Dynamics(2,2)

    rrt = RRT(start, goal, obstacle_list, sample_domain, dynamics, n_samples=n_samples)

    path = rrt.planPath(animation=True)

    print("Done")


if __name__ == "__main__":
    main()