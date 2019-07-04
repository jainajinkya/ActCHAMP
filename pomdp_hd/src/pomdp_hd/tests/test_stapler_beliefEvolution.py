#!/usr/bin/env python

import numpy as np
import copy
from pomdp_hd.cython.beliefEvolution import *

def main():

    start_mu = np.array([92., -5., 16., -112.]) # start in ds state: 2
    start_x = copy.copy(start_mu)
    
    goal = np.array([92., -5., -25., -175.]) # Goal State: 2

    dynamics = BeliefEvolution()
    dynamics.nSamples = 50.
    dynamics.ds_res_loop_count_ = 1
    dynamics.goal_threshold = 0.02*100.
    dynamics.setGoalGC(copy.copy(goal))  # Setting GC

    ## Checking Guard Conditions
    # goals = [np.array([0.80, -0.04, 0.25, -0.5])*100.,
    #         np.array([0.85, -0.04, 0.01, -0.5])*100.,
    #         np.array([0.85, -0.04, 0.25, 1.57])*100.,
    #         np.array([0.85, -0.04, 0.09, 0.80])*100., 
    #         np.array([0.60, 0.09, -0.20, -0.05])*100.]

    # cov = 1e-6*np.eye(4)

    # for i, goal2 in enumerate(goals):
    #     wts = np.ndarray(dynamics.nModel)
    #     dynamics.fastWtsMapped(goal2, cov, wts)
    #     print "\nGoal: ", goal2, "\nIdeal idx: ", i, "\tActual idx: ", np.argmax(wts), "\tActual wts: ", wts


    # x = np.array([-10.45917503, 49.81662176, -7.53142689, 21.19072645])
    # u = np.zeros(3)


    # nState = dynamics.nState
    # nInput = dynamics.nInput
    # A = np.ndarray((nState, nState))
    # B = np.ndarray((nState, nInput))
    # C = np.ndarray((dynamics.nOutput, dynamics.nOutput))
    # V = np.ndarray((dynamics.nState, dynamics.nState))
    # W = np.ndarray((dynamics.nOutput, dynamics.nOutput))

    
    # cov = 1e-6*np.eye(nState)
    # wts = np.ndarray(dynamics.nModel)
    # dynamics.fastWtsMapped(x, cov, wts)
    # idx = np.argmax(wts)
    # print "Curent idx: ", idx

    # dynamics.getMatrices(idx, x, u, A, B, C, V, W)

    # print "System Matrices:\n", 
    # print "A_mat: ", A
    # print "B_mat: ", B
    # print "C_mat: ", C
    # print "V_mat: ", V
    # print "W_mat: ", W


    ## Test Dynamics
    nSegments = 500
    tFinal = 2
    mu = copy.copy(start_mu)
    cov = copy.copy(50.0*np.eye(4))
    u = (goal[:3]-mu[:3])/(nSegments)
    wts = np.ndarray(5)
    ds = 0
    
    print "Start_mu: ", start_mu
    print "Goal:", goal
    print "\n Applied Control: ", u

    for i in range(nSegments):
        ds_bar = dynamics.beliefUpdatePlanning(mu, cov, u, mu, cov, wts, ds)
        print "\nStep: ", i+1
        print "mu :", np.round(mu,3)
        print "cov: ", np.round(cov, 3)
        print "wts:", wts

if __name__ == "__main__":
    main()
