#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import sys
import time
from contextlib import contextmanager
from itertools import izip, permutations
from multiprocessing import Process, Pipe
import numpy as numpy
import scipy.interpolate
from scipy.optimize import differential_evolution

from pomdp_hd.cython.beliefEvolution import *
from pomdp_hd.src.py_utils import *
from pomdp_hd.src.belief_rrt import *


## Global Parameters on which optimizer to use
USE_SNOPT = True

if USE_SNOPT:
    snopt_call = 0
    from pomdp_hd.src.trajOpt_snopt import *
else:
    from pomdp_hd.src.trajOpt import *


"""
Parallel Processing
"""
@contextmanager # For Making it to work with Python 2.7
def terminating(thing):
    """
    Function to handle termination condition for Parllel threads
    """
    try:
        yield thing
    finally:
        thing.terminate()

def spawn(f):
    """
    High level funciton to use parallel processing with classes
    """
    def fun(pipe,x):
        pipe.send(f(x))
        pipe.close()
    return fun

def parmap(f,X):
    """
    Function to map functions to parallel threads
    """
    pipe=[Pipe() for x in X]
    proc=[Process(target=spawn(f),args=(c,x)) for x,(p,c) in izip(X,pipe)]
    [p.start() for p in proc]
    [p.join() for p in proc]
    return [p.recv() for (p,c) in pipe]


""" Custom Data Structures """
class hybrid_belief(object):
    """
    Data structure to represent hybrid belief
    
    Fields:
        mu  : Numpy Array
            Mean of Gaussian distribution over continuous state
        cov : Numpy Matrix
            Covariance of Gaussian distribution over continuous state
        wts : Numpy Array/List
            Probability vector over discrete states 
    """
    def __init__(self, mu, cov, wts):
        self.mu = mu
        self.cov = cov
        self.wts = wts


class data_info(object):
    """
    Data structure to record values associated with a High-level plan or rollout
    
    Fields:
        rollout : Numpy Array/List
            High-Level Plan
        cost : double
            Cost in following this high-level plan
        plan : Numpy Array of Numpy Arrays
            Continuous state plan corresponding to this rollout
        final_wts : Numpy Array/List
            Proabibilty vector over discrete states at the end of this roolout
        cs_goal_path: list of Numpy arrays
            Intermediate goals to go to in continuous states
        """

    def __init__(self, rollout, cost, plan, final_wts, cs_goal_path):
        self.rollout = rollout
        self.cost = cost
        self.plan = plan
        self.final_wts = final_wts
        self.cs_goal_path = cs_goal_path


""" Helper Functions """
def stichCSPath(planned_path):
    """
    Function to stich togther multiple continuous state plans into a single plan

    Input
    =====
    planned_path : List/numpy Array
        List of individual conitnuous state plans

    Output
    ======
    complete_path : Numpy Array of Arrays
        Single stiched path
    """
    complete_path = planned_path[0]

    for i, plan in enumerate(planned_path[1:]):
            complete_path[0] = np.append(complete_path[0], plan[0], axis=1)
            complete_path[1] = np.append(complete_path[1], plan[1], axis=1)
            complete_path[2] = np.append(complete_path[2], plan[2], axis=1)
            # if self.use_RRT:
                # complete_path[0] += plan[0]
                # complete_path[1] += plan[1]
                # complete_path[2] += plan[2]

    return complete_path


class pomdp_hd:
    def __init__(self):
        ## PARAMETERS
        self.do_parallelize = True
        self.do_verbose = False
        self.show_traj = True
        self.first_pass = True
        self.do_global_optimization = False
        self.direct_planning = False

        ## Initalize Dynamics definition  
        self.dyna = BeliefEvolution()

        self.nState = self.dyna.nState
        self.nInput = self.dyna.nInput
        self.nOutput = self.dyna.nOutput
        self.nModel = self.dyna.nModel
        
        # raw_input('Did you set up domain for global optimization?\n Press enter to continue!')
        self.domain = [(-5.0, 5.0)]*self.nState

        # Hybrid Belief
        self.start_mu = np.array([0.]*self.nState)
        self.start_cov = 0.5*np.eye(self.nState)
        wts = np.ndarray(self.nModel)
        self.start_ds = 0
        self.goal_cs = np.array([0.5]*self.nState)
        self.goal_wts = np.ndarray(self.nModel)
        self.goal_ds = 0
        self.belief = hybrid_belief(self.start_mu, self.start_cov, wts)
        self.id1 = np.triu_indices(self.nState)

        ## Trajectory Optimization
        ## PlANNING TIME HORIZON
        self.T = 2.
        self.nSegments = 10
        self.opt = self.setTrajectoryOptimization()

        # rbf result
        self.rbf = None
        self.rollouts = []
        self.set_of_ds_goals = []
        self.scale = 100.

        ## Problem Speicifc Definition
        self.allowed_ds_transitions = [[0,1], [1, 0], [1,2], [2,1], [2,3]]

        ## Final Traejctory
        self.extra_traj_res = 3

        print "POMDP-HD Set"

    def setTrajectoryOptimization(self):
        # Cost Matrices
        Q = 5.*np.eye(self.nState)
        R = 1.*np.eye(self.nInput)
        Q_f = 20.*np.eye(self.nState)
        labda = 0e0*np.eye((self.nState*(self.nState+1)/2))
        # labda[1,1] = 0.

        opt = trajectoryOptimization(self.nState, self.nSegments, self.nInput, self.nOutput, self.nModel, Q, R, Q_f, labda, self.dyna)

        return opt


    def plan_optimal_path(self):
        print "In Planner, \nstart_mu: ", self.start_mu, "\nstart_cov:", self.start_cov
        # raw_input('Press Enter to Begin!')
        self.find_set_of_ds_to_cs_goals()
        return self.find_optimal_path()  


    def find_optimal_path(self):       
        all_plans = []
        mu_safe = copy.copy(self.belief.mu)
        cov_safe = copy.copy(self.belief.cov)
        wts_safe = copy.copy(self.belief.wts)

        if self.direct_planning:
            mu_plan, s_plan, u_plan, final_wts = self.optimal_cs_path(self.belief, self.goal_cs)
            return mu_plan, s_plan, u_plan

        else:         
            self.belief.mu = copy.copy(mu_safe)
            self.belief.cov = copy.copy(cov_safe)
            self.belief.wts = copy.copy(wts_safe)

            # convert goal in continuous states to ds goal
            cov = 1e-6*np.eye(self.nState)
            self.dyna.fastWtsMapped(self.goal_cs, cov, self.goal_wts)
            self.goal_ds = copy.copy(self.wts2ds(self.goal_wts))

            ''' Do rollouts to find a feasible lower cost path  '''
            # self.generate_rollouts()
            self.rollouts = [[0,1,2,3,4]]
            # raw_input('generated rollouts. Press Enter to continue!')

            ''' Calculate cost for the rollout'''
            # ## Convert rollout to wts
            if self.do_parallelize:
                all_plans = parmap(self.rollout_cost, self.rollouts)
            else:
                all_plans = []
                for rollout in self.rollouts:
                    rollout_data = self.rollout_cost(rollout)
                    all_plans.append(rollout_data)

            min_cost_path = self.optimal_ds_path(all_plans)
            # print "min_cost_path: \n", min_cost_path.plan[0]

            # """ Generate last part to go to goal """
            ## Using TrajOpt
            self.belief.mu = copy.copy(min_cost_path.plan[0][:,-1])
            self.belief.cov[self.id1] = copy.copy(min_cost_path.plan[1][:,-1])
            self.dyna.fastWtsMapped(self.belief.mu, self.belief.cov, self.belief.wts)
            cost, mu_plan, s_plan, u_plan, final_wts = copy.copy(self.opt.cs_optimize(self.belief.mu, self.belief.cov, self.belief.wts, self.T, self.goal_cs))
            mu_plan, s_plan, u_plan = copy.copy(self.generateDetailedCSPlan([[mu_plan],[s_plan],[u_plan]]))
            
            # # Appending the final plan
            min_cost_path.plan[0] = np.append(min_cost_path.plan[0], mu_plan, axis=1)
            min_cost_path.plan[1] = np.append(min_cost_path.plan[1], s_plan, axis=1)
            min_cost_path.plan[2] = np.append(min_cost_path.plan[2], u_plan, axis=1)
            plan_path = [min_cost_path.plan[0], min_cost_path.plan[1], min_cost_path.plan[2]]

            ## Using RRT
            # mu_plan, s_plan, u_plan, final_wts = self.optimal_cs_path(self.belief, self.goal_cs)
           
            # min_cost_path.plan[0] += mu_plan
            # min_cost_path.plan[1] += s_plan
            # min_cost_path.plan[2] += u_plan

            print "*****************************************"
            print "min_cost_path: \n", min_cost_path.rollout, "\ncs goals: ", min_cost_path.cs_goal_path# ,"\ncost :", min_cost_path.cost, "\ncs plan, mu: ", min_cost_path.plan[0].T, "\ncs plan, cov: ", min_cost_path.plan[1],  "\ncs plan, u: ", min_cost_path.plan[2]
            print "*****************************************"


            # return self.generateDetailedCSPlan(plan_path)

            if self.show_traj:
                # self.plot_trajectory(min_cost_path) ## LQR
                self.plot_trajectory(plan_path)


            # return min_cost_path[0], min_cost_path[1], min_cost_path[2]
            return min_cost_path.plan[0], min_cost_path.plan[1], min_cost_path.plan[2]



    def generate_rollouts(self):
        self.rollouts = []

        # all_possible = range(1, self.nModel+1)
        all_possible = range(self.nModel)

        self.end_ds = self.wts2ds(self.goal_wts)
        print "end_ds = ", self.end_ds, ", with wts: ", self.goal_wts
        all_possible.remove(self.end_ds) # remove goal state

        # removing start ds
        wts = np.ndarray(self.nModel)
        self.dyna.fastWtsMapped(copy.copy(self.start_mu), copy.copy(self.start_cov), wts)
        self.start_ds = copy.copy(self.wts2ds(wts))
        print "Start mu: ", self.start_mu
        print "start_ds =", self.start_ds, ", with wts: ", wts

        # if self.start_ds != self.end_ds:
        #     all_possible.remove(self.start_ds)
        # else:
        #     self.rollouts= [[self.start_ds, self.end_ds]]
        #     return

        # Generate all rollouts:
        for L in range(1, len(all_possible)+1):
            for subset in permutations(all_possible, L):
                subset = list(subset)
                subset.insert(0, self.start_ds)
                subset.append(self.end_ds)

                if self.is_rollout_feasible(subset):
                    self.rollouts.append(subset)

        print "All possible rollouts : ", self.rollouts

        return

    def is_rollout_feasible(self, rollout):
        for i in range(len(rollout)-1):
            if [rollout[i], rollout[i+1]] not in self.allowed_ds_transitions:
                return False
        return True

    def rollout_cost(self, rollout):
        ds_goal_path = self.dsArray2wtsArray(rollout)
        # We don't need the first ds goal

        if self.do_verbose:
            print "ds_goal_path = ", ds_goal_path

        self.belief.mu = copy.copy(self.start_mu)
        self.belief.cov = copy.copy(self.start_cov)

        # convert start in continuous states to ds
        self.dyna.fastWtsMapped(self.belief.mu, self.belief.cov, self.belief.wts)

        start_wts = copy.copy(self.belief.wts)

        if self.do_verbose:
            print "Initial Belief = ", self.belief.mu, self.belief.cov, self.belief.wts

        # convert path to continuous states
        cs_goal_path = self.ds_traj_to_cs_traj(ds_goal_path)
        if self.do_verbose:
            print "cs_goal_path = ", cs_goal_path

        # raw_input('Check CS goal path')

        # Find optimal path for all continuous state goals
        planned_path = self.complete_optimal_cs_traj(cs_goal_path)
        if self.do_verbose:
            print "planned_path = \n", planned_path

        # Calculate associated cost
        final_wts = planned_path[-1][-1]

        # final_wts = self.belief.wts*1.
        cost = smooth_cost(self.goal_wts, final_wts, cost_type="KL", scale=1e3)

        # Adding cost for executing longer path
        cost += 0e3*self.T*len(ds_goal_path)

        rollout_in_ds = [self.start_ds] + self.wtsArray2dsArray(ds_goal_path)

        # raw_input('rollout cost Generated. Press Enter to continue!')

        return data_info(rollout_in_ds, cost, stichCSPath(planned_path), final_wts, cs_goal_path)


    def optimal_ds_path(self, all_plans):
        global max_cost_differences
        max_cost_differences = []
        cost_diff = []
        
        min_cost = 10e6
        min_cost_plan = None

        for plan in all_plans:
            # print "For rollout: ", plan.rollout , "Cost = ", plan.cost, "final wts = ", np.round(plan.final_wts,3), "cs_goal_path", plan.cs_goal_path

            if plan.cost < min_cost:
                min_cost = plan.cost*1.
                min_cost_plan = copy.copy(plan)

        max_cost_differences.append(cost_diff)
        # print "min_cost_plan", min_cost_plan.plan
        return min_cost_plan


    def ds_goal_to_cs_goal(self, ds_goal):
        return self.set_of_ds_goals[self.wts2ds(ds_goal)]


    def find_set_of_ds_to_cs_goals(self):
        if self.do_global_optimization:
            for i in range(self.nModel-1):
                ds_goal = [0.]*self.nModel
                ds_goal[i] = 1.

                wts = [0.]*self.nModel
                wts[i-1] = 1.

                while(self.wts2ds(ds_goal) != self.wts2ds(wts)):
                    # generate ds_costmap
                    self.ds_costmap(ds_goal)

                    # do global optimization
                    res = differential_evolution(self.global_objective, self.domain)

                    # return cs goal
                    if self.do_verbose:
                        wts = np.ndarray(self.nModel)
                        self.dyna.fastWtsMapped(res.x, 0.01*np.eye(self.nState), wts)
                        print "result = ", res.x
                        print "Target wts : ", ds_goal
                        print "Wts at final Point : ", np.round(wts ,3)


                print "\nTarget ds: ", self.wts2ds(ds_goal), " CS Goal: ", res.x
            
                # projected_goal = copy.copy(self.project_on_boundary(self.wts2ds(ds_goal), res.x))
                # self.set_of_ds_goals.append(projected_goal)
                self.set_of_ds_goals.append(res.x)

        else:
            ## We are predefining the set of continuous states that planner should go through
            # self.set_of_ds_goals.append(np.array([80., 0., 125., 78.]))
            # self.set_of_ds_goals.append(np.array([100., 0., 98., 78.]))
            # self.set_of_ds_goals.append(np.array([100., 0., 125., 257.]))
            # self.set_of_ds_goals.append(np.array([100., 0., 98., 257.]))

            self.set_of_ds_goals.append(np.array([0.92, -0.05, 0.16, -1.12])*self.scale)
            # self.set_of_ds_goals.append(np.array([0.92, -0.05, -0.10, -1.75])*self.scale)
            self.set_of_ds_goals.append(np.array([0.92, -0.05, -0.10, -1.35])*self.scale)
            self.set_of_ds_goals.append(np.array([0.92, -0.05, -0.14, -2.22])*self.scale)
            self.set_of_ds_goals.append(np.array([0.92, -0.05, 0.30, -2.22])*self.scale)


        self.set_of_ds_goals.append(self.goal_cs)
        print "set of continuous state Goals: \n" , self.set_of_ds_goals


    def ds_traj_to_cs_traj(self, ds_traj):
        cs_traj = []
        for ds_goal in ds_traj:
            cs_goal = self.ds_goal_to_cs_goal(ds_goal)
            cs_traj.append(cs_goal)

        return cs_traj


    def optimal_cs_path(self, start, cs_goal):
        # do snopt based optimization to find the optimal trajectory
        muInit = start.mu
        covInit = start.cov
        wtsInit = start.wts

        start_time2 = time.time()
        cost, mu_plan, s_plan, u_plan, final_wts = self.opt.cs_optimize(muInit, covInit, wtsInit, self.T,  cs_goal)
        
        print "#############################################"
        print "Total Time in one call of SNOPT = %s seconds" % (time.time() - start_time2)
        global snopt_call
        snopt_call += 1
        print "SNOPT call number = ", snopt_call
        print "#############################################"

        # # raise NameError('Debug')

        # dynamics = BeliefDynamics(cs_goal)
        # sample_domain = [[72., 80.],[-20., 20.],[-25., 42.],[-50., 170.]]
        # n_samples = 200
        # obstacle_list = []
        # rrt = RRT(start.mu, cs_goal, obstacle_list, sample_domain, dynamics, start_cov=start.cov, n_samples=n_samples)
        # mu_plan, s_plan, u_plan = rrt.planPath(animation=False, use_prebuilt_tree=False)

        # final_wts = np.ndarray(dynamics.bel.nModel)
        # cov = copy.copy(start.cov)
        # cov[rrt.COV_IDS] = s_plan[-1]
        # dynamics.bel.fastWtsMapped(mu_plan[-1], cov, final_wts)
        
        return mu_plan, s_plan, u_plan, final_wts


    def complete_optimal_cs_traj(self, cs_traj):
        path = []

        for goal in cs_traj:
            mu_plan, s_plan,u_plan, final_wts = copy.copy(self.optimal_cs_path(self.belief, goal))
            # mu_plan, s_plan, u_plan = copy.copy(self.generateDetailedCSPlan([[mu_plan],[s_plan],[u_plan]]))
            optimal_path =  [mu_plan, s_plan, u_plan, final_wts]

            if self.do_verbose:
                print "Optimal path: \n", optimal_path

            path.append(optimal_path)

            # ## Next Iteration RRT
            # self.belief.mu = copy.copy(optimal_path[0][-1])
            # self.belief.cov[self.id1] = copy.copy(optimal_path[1][-1]) # Conversion of s to cov
            # self.belief.wts = copy.copy(final_wts)


            # Next Iteration TrajOpt
            self.belief.mu = copy.copy(optimal_path[0][:,-1])
            self.belief.cov[self.id1] = copy.copy(optimal_path[1][:,-1]) # Conversion of s to cov
            self.belief.cov = 2*np.eye(self.nState) # Conversion of s to cov
            self.belief.wts = copy.copy(final_wts)
            self.dyna.fastWtsMapped(self.belief.mu, self.belief.cov, self.belief.wts)

            if self.do_verbose:
                print "New Start point:\nmu: ", self.belief.mu, "\ncov: ", self.belief.cov, "\nwts: ", self.belief.wts
            print "Optimal path: \n", np.round(mu_plan.T,3)
            print "\n Input: \n", np.round(u_plan.T,3)

            # raw_input('Press Enter if should go to next cs_goal')

        return path


    ## Other FUNCTIONS

    def project_on_boundary(self, ds_goal, res):
        # Ideally can use another optimization process to find the nearest point, but here we can use direct formulae
        projection = deepcopy(res)
        if ds_goal == 1:
            projection[1] = -20.
        elif ds_goal == 2:
            projection[0] = -20.

        return projection

    def ds_costmap(self, goal_wts):
        # Local Parameters
        cov = 25.0*np.eye(self.nState)
        wts = np.ndarray(self.nModel)

        # Generate data:
        pts = np.random.random((self.nState,1000))

        # Scaling data based on the domain size
        for i in range(self.nState):
            pts[i,:] *= (self.domain[i][1] - self.domain[i][0])
            pts[i,:] += self.domain[i][0]

        costs = []
        for pt in pts.T:
            self.dyna.fastWtsMapped(pt*1., cov, wts)
            # cost = copy.copy(smooth_cost(goal_wts, wts, cost_type="Hellinger", scale=1e3)) # cost for ds mismatch
            cost = copy.copy(smooth_cost(goal_wts, wts, cost_type="sym_KL", scale=1e3)) #
            # cost += 5e2*numpy.linalg.norm(self.goal_cs - pt)
            
            # if (np.linalg.norm(goal_wts - np.array([0., 0., 0., 1.])) > 1.0): 
            #     # cost += 50.*numpy.linalg.norm(copy.copy(np.array([-140.0, -20.0])) - pt)
            #     cost += 10.*numpy.linalg.norm(copy.copy(self.goal_cs) - pt)
            # else:
            #     cost += 500.*numpy.linalg.norm(copy.copy(self.goal_cs) - pt)


            costs.append(cost)

        costs = np.array(costs)

        # Interpolate
        x = pts[0,:]
        y = pts[1,:] # - 6*np.ones(len(pts.T))
        self.rbf = scipy.interpolate.Rbf(x, y, costs, function='multiquadric')
        return  

    def global_objective(self, X):
        return self.rbf(X[0], X[1])

    def ds2Wts(self, ds_state):
        wts = np.zeros(self.nModel)
        # wts[ds_state-1] = 1.
        wts[ds_state] = 1. 
        return wts

    def dsArray2wtsArray(self, dsArray):
        wtsArray = []
        for ds in dsArray:
            wtsArray.append(self.ds2Wts(ds))
        return wtsArray

    def wts2ds(self, wts):
        # return int(np.argmax(wts) + 1) 
        return int(np.argmax(wts)) 


    def wtsArray2dsArray(self, wtsArray):
        dsArray = []
        for wts in wtsArray:
            dsArray.append(self.wts2ds(wts))
        return dsArray


    def generateDetailedCSPlan(self, input_plan):
        mu_in = input_plan[0][0]
        s_in = input_plan[1][0]
        u_in = input_plan[2][0]

        # New matrices
        mu_plan = np.zeros((self.nState, self.nSegments*self.extra_traj_res))
        s_plan = np.zeros((self.opt.len_s, self.nSegments*self.extra_traj_res))
        u_plan = np.zeros((self.nInput, self.nSegments*self.extra_traj_res))

        mu_plan[:,0] = copy.copy(mu_in[:,0])
        s_plan[:,0] = copy.copy(s_in[:, 0])

        mu = copy.copy(mu_plan[:,0])
        cov = symarray(np.zeros((self.nState, self.nState)))
        cov[self.opt.id1] = copy.copy(s_plan[:,0])

        mu_new  = np.ndarray(self.nState)
        w_new = np.ndarray(self.nModel)
        ds = 0
        wts = np.ndarray(self.nModel)

        for i in range(self.nSegments-1):
            for t in range(self.extra_traj_res):
                ds = self.dyna.beliefUpdatePlanning(mu, cov, u_in[:,i]/self.extra_traj_res, mu_new, cov, wts, ds)

                mu_plan[:,i*self.extra_traj_res+t] = copy.copy(mu_new)
                s_plan[:,i*self.extra_traj_res+t] = copy.copy(cov[self.opt.id1])
                u_plan[:,i*self.extra_traj_res+t-1] = copy.copy(u_in[:,i])

                mu = copy.copy(mu_new)
                
                # if self.do_verbose:
                print "\n Planned Step", i*self.extra_traj_res+t
                print "Applied Control, u_plan = ", np.round(u_plan[:,i*self.extra_traj_res+t-1], 2)
                # print "Planned mu = ", np.round(mu_plan[:,i*self.extra_traj_res+t], 3)
                print "Planned mu = ", np.round(mu_new, 3)

                print "Planned s = ", np.round(s_plan[:,i*self.extra_traj_res+t], 3)
                print "Planned Wts: ", wts.T

        return mu_plan, s_plan, u_plan


    def plot_trajectory(self, path):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        sample_domain = [[60., 100.],[-50., 50.],[-20., 50.],[-300., 100.]]

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

            ax.scatter(path_x[0], path_y[0], path_z[0], c="k", marker="s", s=150)
            ax.scatter(path_x[-1], path_y[-1], path_z[-1], c="g", marker="s", s=150)

            ax.set_xlim(sample_domain[0][0], sample_domain[0][1])
            ax.set_ylim(sample_domain[1][0], sample_domain[1][1])
            ax.set_zlim(sample_domain[2][0], sample_domain[2][1])

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
            plt.ylim([sample_domain[3][0], sample_domain[3][1]])

            plt.show()

        # print "#############################################"
        # print "\n\n\n\nFinal Trajecotry"
        # traj = []
        # for plan in path:
        #     traj_dummy = np.array(plan[0]).T
        #     for i in range(len(traj_dummy)):
        #         print "Planned Trajectory", traj_dummy[i]
        #         traj.append(traj_dummy[i])
        #         # print "True Trajectory", traj_true[i]

        # print "#############################################"

        # ### Plotting
        # fig = plt.figure(1)

        # ### Setting Up Domain
        # x1 = 3.0
        # y1 = self.domain[1][0]
        # x2 = 3.0
        # y2 = self.domain[1][1]
        # plt.plot([x1, x2], [y1, y2], color='b', linestyle='--', linewidth=2)

        # x1 = 6.0
        # y1 = self.domain[1][0]
        # x2 = 6.0
        # y2 = self.domain[1][1]
        # plt.plot([x1, x2], [y1, y2], color='b', linestyle='--', linewidth=2)

        # x1 = 3.0
        # y1 = -2.0
        # x2 = 6.0
        # y2 = -2.0
        # plt.plot([x1, x2], [y1, y2], color='b', linestyle='--', linewidth=2)

        # x1 = 3.0
        # y1 = 3.0
        # x2 = 6.0
        # y2 = 3.0
        # plt.plot([x1, x2], [y1, y2], color='b', linestyle='--', linewidth=2)


        # x_vec = [traj[i][0] for i in range(len(traj))]
        # y_vec = [traj[i][1] for i in range(len(traj))]
        # # x_vec_true = [traj_true[i][0] for i in range(len(traj_true))]
        # # y_vec_true = [traj_true[i][1] for i in range(len(traj_true))]

        # bl1, = plt.plot(x_vec,y_vec, 'k-o', linewidth = 2.0)
        # # bl2, = plt.plot(x_vec_true,y_vec_true, 'b-D',linewidth = 3.0)

        # plt.title('Trajectories for Discrete state based planning')
        # # plt.legend([bl1, bl2], ['Planned Trajectory', 'Actual Trajectory'])

        # # img = imread("domain/test_domain6.png")
        # # # img1 = ndimage.rotate(img, 90)
        # # img1 = np.flipud(img)
        # # plt.imshow(img1, zorder=0, extent=[0., 10.0, 0.0, 10.0])
        # # axes = plt.gca()
        # # Start Point
        # plt.plot(x_vec[0], y_vec[0], 'ko', x_vec[-1], y_vec[-1], 'gs' , ms=10.0, mew=2.0)
        # # plt.plot(x_vec_true[0], y_vec_true[0], 'ko', x_vec_true[-1], y_vec_true[-1], 'gs' , ms=10.0, mew=2.0)   
        # # axes.set_xlim([xmin,xmax])
        # # axes.set_ylim([-2.0, 3.0])
        # fig.show()
        # raw_input()

        # return



if __name__ == "__main__":
    planner = pomdp_kc()

    planner.dyna.nSamples = 10
    mu = np.array([2.0, 0.5])
    cov = 5*np.eye(planner.nState)
    wts = np.ndarray(planner.nModel)
    planner.belief = hybrid_belief(mu, cov, wts)
    goal = np.array([5.0, 0.0])

    # planner.rollouts = [[1,3], 
    # planner.rollouts = [[1, 5, 3], [1, 2, 3]] #, [1, 4, 3], [1, 2, 5, 3], [1, 4, 5, 3], [1, 2, 4, 5, 3]]

    start_time = time.time()
    planner.start_mu = mu
    planner.start_cov = cov
    planner.goal_cs = goal
    planner.do_parallelize = True
    planner.plan_optimal_path()

    if planner.do_parallelize:
        print "Total Time spent with paraellization = %s seconds" % (time.time() - start_time)
    else:
        print "Total Time spent without paraellization = %s seconds" % (time.time() - start_time)

    global max_cost_differences
    print "cost diffrences", max_cost_differences


    '''
    #######Timing for nSamples Parameter##########
    n_iter = 5
    # Samples = 100
    planner.dyna.nSamples = 100
    start_time = time.time()
    for i in range(n_iter):
        mu = np.array([2.0, 0.5])
        cov = 5*np.eye(planner.nState)
        wts = np.ndarray(planner.nModel)
        planner.belief = hybrid_belief(mu, cov, wts)

        goal = np.array([10.0, 0.])
        planner.find_optimal_path(mu, cov, goal)

    print "N Samples for dynamics: ", planner.dyna.nSamples
    print "Average Time spent in Planning = %s seconds" % ((time.time() - start_time)/n_iter)
    '''

    # planner.belief.mu = np.array([  4.76, -14.52] )
    # # planner.belief.cov = 5.0*np.eye(planner.nState)
    # planner.belief.cov = np.array([[ 0.0310559,  0.],[ 0.,0.0310559]])
    # planner.belief.wts = np.array([ 0.,  1.,  0.,  0.,  0.])
    # goal = np.array([ 4.67, -0.16 ])

    # planner.optimal_cs_path(planner.belief, goal)

    ''' Checking PD functionalties

    # cov = np.array([[ 16.75690398,  10.94250939], [ 10.94250939 ,  7.14562259]])
    # res = np.ndarray((planner.nState, planner.nState))
    # planner.dyna.nearestPDMapped(cov, res)

    # print "Input Matrix = ", cov
    # print "res =", res
    # print "isPD = ", planner.dyna.isPDMapped(res)
    '''
