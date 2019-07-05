#!/usr/bin/env python

import numpy as np
import copy
from lqr_rrt_star import RRT, Node, LQR
from pomdp_hd.cython.beliefEvolution import *


class BeliefDynamics(object):
    u_threshold = 10.

    def __init__(self, goal):
        self.setDynamics(goal)
        self.nState = self.bel.nState 
        self.nInput = self.bel.nInput
        self.A = np.ndarray((self.nState, self.nState))
        self.B = np.ndarray((self.nState, self.nInput))
        self.C = np.ndarray((self.bel.nOutput, self.bel.nOutput))
        self.V = np.ndarray((self.bel.nState, self.bel.nState))
        self.W = np.ndarray((self.bel.nOutput, self.bel.nOutput))
        self.Q = 0.5*np.eye(self.nState)
        self.R = 0.5*np.eye(self.nInput)
        self.LQR = LQR()

    def setDynamics(self, goal):
        self.bel = BeliefEvolution()
        self.bel.nSamples = 50.
        self.bel.ds_res_loop_count_ = 1
        self.bel.goal_threshold = 0.02*100.
        self.bel.setGoalGC(goal)  # Setting GC

    def A_mat(self, x, u=None):
        if u is None:
            u = np.zeros(self.nInput)

        idx = self.getIdx(x)
        self.bel.getMatrices(idx, x, u, self.A, self.B, self.C, self.V, self.W)
        return self.A

    def B_mat(self, x, u=None):
        if u is None:
            u = np.zeros(self.nInput)

        idx = self.getIdx(x)
        self.bel.getMatrices(idx, x, u, self.A, self.B, self.C, self.V, self.W)
        return self.B

    def Q_mat(self):
        return self.Q

    def R_mat(self):
        return self.R

    def getIdx(self, x, cov=1e-3*np.eye(4)):
        wts = np.ndarray(self.bel.nModel)
        self.bel.fastWtsMapped(x, cov, wts)
        return np.argmax(wts)

    def LQR_mats(self, x, u=None):
        A = self.A_mat(x, u)
        B = self.B_mat(x, u)
        Q = self.Q_mat()
        R = self.R_mat()
        # print "B_mat: ", B
        return self.LQR.dlqr(A, B, Q, R)

    def LQR_control(self, x):
        Kopt, S, ev = self.LQR_mats(x)
        u = -1.*Kopt.dot(x)

        ## Thresholding u
        for v in u:
            if abs(v) > self.u_threshold:
                factor = self.u_threshold/abs(v)
                u *= factor

        return u

    def propagateState(self, x, u, cov=1e-3*np.eye(4)):
        x_new = np.ndarray(self.nState)
        # cov = 1e-3*np.eye(self.nState)
        wts = np.ndarray(self.bel.nModel)
        ds = 0
        # ds = self.bel.beliefUpdatePlanning(x, cov, u, x_new, cov, wts, ds)

        ## Numerical Integration
        integration_steps = 20.
        u_new = u/integration_steps

        for i in range(int(integration_steps)):
            ds = self.bel.beliefUpdatePlanning(x, cov, u_new, x_new, cov, wts, ds)
            x = copy.copy(x_new)

        return x_new, cov



def main():
    start = np.array([85.538, -50.0, 25.462, 157.])
    start_cov = 50*np.eye(4)
    # start = np.array([64.01452033, 2.14287008, -6.05454833, -2.84716919])
    goals = [
             np.array([85., -4., 16.0, 157.]),
             # np.array([85., -4., 7.2, 90.]),
             # np.array([85., -4., 0., 0.]),
             # np.array([85., -4., 30.0, 0.])
             ]


    sample_domain = [[-100., 100.],[-100., 100.],[-100., 100.],[-100., 200.]]
    n_samples = 100
    obstacle_list = []

    full_path = []

    for i, goal in enumerate(goals):
        dynamics = BeliefDynamics(goal)
        rrt = RRT(start, goal, obstacle_list, sample_domain, dynamics, start_cov=start_cov, n_samples=n_samples)
        path, s_path, u_path = rrt.planPath(animation=False, use_prebuilt_tree=False)

        if path is None:
            print "FAILED TO FIND ANY PATH"
        else:
            print "BEST PATH RETURNED"

        full_path += path

        if i < len(goals)-1 and len(path)>0:
            # raw_input("\nPress Enter to go to next goal! ")
            start = path[-1]

    print "\nPlotting final path!\n"
    rrt.plotPath(full_path)


if __name__ == "__main__":
    main()