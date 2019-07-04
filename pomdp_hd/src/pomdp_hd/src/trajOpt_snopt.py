#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  7 14:55:59 2017

@author: Ajinkya
"""

import time, sys

import numpy as np
from scipy.optimize import *
from optimize import snopta, SNOPT_options
import copy
from numpy import linalg as la

from py_utils import *

class trajectoryOptimization:
    MAX_TRIES = 5

    def __init__ (self, nState, nSegments, nInput, nOutput, nModel, Q, R, Q_f, labda, sysDynamics):
        # Planning Parameters
        self.nState = nState
        self.nInput = nInput
        self.nOutput = nOutput
        self.nSegments = nSegments
        self.nModel = nModel
        self.len_s = self.nState*(self.nState+1)/2
        self.id1 = np.triu_indices(self.nState)
        self.goal = None
        self.delta = 1
        self.sysDynamics = sysDynamics
        self.extra_euler_res = 20

        # Cost Matrices
        self.Q = Q
        self.R = R
        self.Q_f = Q_f
        self.labda = labda

        # Logging
        self.do_verbose = True


    def objectiveFunction(self, X):
        s = X[(self.nState + self.len_s)*self.nSegments - self.len_s:(self.nState + self.len_s)*self.nSegments]
        J = 0.

        for i in range(self.nSegments-1):
            m = X[i*self.nState:(i*self.nState+self.nState)]
            u = X[i*self.nInput + (self.nState + self.len_s)*self.nSegments: (i+1)*self.nInput + (self.nState + self.len_s)*self.nSegments]
            J = J + (m-self.goal).dot(self.Q).dot(m-self.goal) + u.dot(self.R.dot(u))
            # J = J + (m[:3]-self.goal[:3]).dot(self.Q[:3, :3]).dot(m[:3]-self.goal[:3]) + u.dot(self.R.dot(u))

        m_T = X[(self.nSegments-1)*self.nState:(self.nSegments-1)*self.nState+self.nState]

        J = J + s.T.dot(self.labda).dot(s) + (m_T-self.goal).dot(self.Q_f).dot(m_T-self.goal)
        # J = J + s.T.dot(self.labda).dot(s) + (m_T[:3]-self.goal[:3]).dot(self.Q[:3, :3]).dot(m_T[:3]-self.goal[:3])

        return J
    

    def constraints(self, X, X0):         
        m_old = copy.copy(X[:self.nState])
        s_old = copy.copy(X[self.nState*self.nSegments:self.nState*self.nSegments + self.len_s])
        U = copy.copy(np.zeros((self.nState, self.nState)))
        U[self.id1] = copy.copy(s_old)
        L = U.T
        cov_old = copy.copy(np.dot(L, L.T.conj()))
        if not is_pos_def(cov_old):
            cov_old = copy.copy(nearestPD(cov_old))

        u_old = X[(self.nState+self.len_s)*self.nSegments:]

        m_new = np.zeros((self.nState,self.nSegments))
        s_new = np.zeros((self.len_s,self.nSegments))
        u_new = np.zeros((self.nInput,self.nSegments-1))
        
        m_new[:,0] = X[:self.nState]
        s_new[:,0] = X[self.nState*self.nSegments:self.nState*self.nSegments + self.len_s]

        u = np.reshape(u_old,(self.nInput,self.nSegments-1),'F')

        ## For Belief Propagation
        mu = np.ndarray(self.nState)
        cov = np.ndarray((self.nState, self.nState))
        wts = np.ndarray(self.nModel)
        ds = 0

        # print "self.delta: ", self.delta
        # print "self.nSegments : ", self.nSegments

        for i in range(self.nSegments-1):
            for t in range(self.extra_euler_res):
                ds_bar = self.sysDynamics.beliefUpdatePlanning(m_old, cov_old, u[:,i]/self.extra_euler_res, mu, cov, wts, ds)

                m_old = copy.copy(mu)
                cov_old = copy.copy(cov)

            m_new[:,i+1] = copy.copy(mu)
            s_new[:,i+1] = copy.copy(cov[self.id1])

        m_new = np.reshape(m_new,(self.nState*self.nSegments),'F')
        s_new = np.reshape(s_new,(self.len_s*self.nSegments),'F')
        u_new = copy.copy(X[(self.nState+self.len_s)*self.nSegments:])

        X_new = np.reshape(np.concatenate((m_new,s_new,u_new),axis=0),len(X),'F')
        return X_new


    def snopt_objFun(self, status, X, needF, F, needG, G):
        obj_f = self.objectiveFunction(X) # Objective Row
        # obj_f = 0.

        cons_f = copy.copy(self.constraints(X, self.X0))

        F[0] = copy.copy(obj_f)
        F[1:] = copy.copy(X)

        cons_f -= X

        # Setting Dynamics for constraints
        F[self.nState+1:self.nState*self.nSegments+1] = cons_f[self.nState:self.nState*self.nSegments]
        F[self.nState*self.nSegments + self.len_s+1:] = cons_f[self.nState*self.nSegments + self.len_s:]

        # print "Objective Function = ", F

        return status, F#, G

    def generate_X0(self, muInit, covInit, goal):
        X0      = np.zeros((self.nState+ self.len_s + self.nInput)*self.nSegments - self.nInput) ## u is needed only till T-1
        
        mu = copy.copy(muInit)
        cov = copy.copy(covInit)
        u = copy.copy((goal[:self.nInput]-mu[:self.nInput])/(self.nSegments-1))
        wts = np.ndarray(self.nModel)
        ds = 0

        X0[:self.nState] = copy.copy(mu)
        X0[self.nState*self.nSegments:self.nState*self.nSegments + self.len_s] = copy.copy(cov[self.id1])
        X0[(self.nState+self.len_s)*self.nSegments:(self.nState+self.len_s)*self.nSegments + self.nInput] = copy.copy(u)

        for i in range(self.nSegments-1):
            ds_bar = self.sysDynamics.beliefUpdatePlanning(mu, cov, u, mu, cov, wts, ds)
            X0[self.nState*(i+1):self.nState*(i+2)] = copy.copy(mu)
            X0[self.nState*self.nSegments + self.len_s*(i+1):self.nState*self.nSegments + self.len_s*(i+2)] = copy.copy(cov[self.id1])

            if i < self.nSegments-2:
                X0[(self.nState+self.len_s)*self.nSegments + self.nInput*(i+1):(self.nState+self.len_s)*self.nSegments + self.nInput*(i+2)] = copy.copy(u)

        return X0

    def cs_optimize(self, muInit, covInit, wtsInit, tFinal, goal):
        # self.delta = int(np.ceil(tFinal/self.nSegments))
        self.delta = tFinal/self.nSegments
        self.sysDynamics.setIntergationStepSize(self.delta)
        self.goal = copy.copy(goal)

        print "\n******************************************************"
        print "Inputs:\n", "mu = ", muInit, "\ncovInit = ", covInit, "\nwtsInit = ", wtsInit, "\ngoal = ", goal
        print "\n******************************************************"


        ##### Setting up to use SNOPT ###
        inf   = 1.0e20
        options = SNOPT_options()

        options.setOption('Verbose',False)
        options.setOption('Solution print',False)
        options.setOption('Print filename','ds_goal_snopt.out')
        options.setOption('Print level',0)

        options.setOption('Optimality tolerance', 1e-2)

        options.setOption('Summary frequency',1)
        options.setOption('Major print level',0)
        options.setOption('Minor print level',0)


        ## ''' INTIALIZATION VECTOR '''
        mu_range = 500.
        s_range = 10000.
        u_range = 10.


        # X0      = np.random.rand((self.nState+ self.len_s + self.nInput)*self.nSegments)
        # X0 = np.concatenate(((np.random.rand(self.nState*self.nSegments)*2. - 1.)*mu_range, (np.random.rand(self.len_s*self.nSegments))*s_range, (np.random.rand(self.nInput*self.nSegments)*2. - 1.)*u_range), axis=0)

        # X0[:self.nState] = copy.copy(muInit)
        # # X0[self.nState*self.nSegments:self.nState*self.nSegments + self.len_s] = covInit[self.id1]*1.


        # '''
        # Do cholesky Factorization on the input matrix
        # '''
        # if not isPD(covInit):
        #     covInit = copy.copy(nearestPD(covInit))

        # L = la.cholesky(covInit).T

        # X0[self.nState*self.nSegments:self.nState*self.nSegments + self.len_s] = np.reshape(L[self.id1] ,(self.len_s,),'F')

        self.X0 = self.generate_X0(muInit, covInit, goal)

        # Xlow    = np.array([-350.0]*len(X0[:self.nState*self.nSegments]) + 
        #           [0.]*len(X0[self.nState*self.nSegments:
        #               (self.nState+self.len_s)*self.nSegments]) + 
        #           [-30.]*len(X0[(self.nState+self.len_s)*self.nSegments:
        #               (self.nState+self.len_s + self.nInput)*self.nSegments]))

        # Xupp    = np.array([350.0]*len(X0[:self.nState*self.nSegments]) +
        #                     [10000.]*len(X0[self.nState*self.nSegments:
        #                     (self.nState+self.len_s)*self.nSegments]) +
        #                 [30.]*len(X0[(self.nState+self.len_s)*self.nSegments:
        #                 (self.nState+self.len_s + self.nInput)*self.nSegments]))

        Xlow    = np.array([-mu_range]*len(self.X0[:self.nState*self.nSegments]) + 
                  [0.]*len(self.X0[self.nState*self.nSegments:
                      (self.nState+self.len_s)*self.nSegments]) + 
                  [-u_range]*len(self.X0[(self.nState+self.len_s)*self.nSegments:]))

        Xupp    = np.array([mu_range]*len(self.X0[:self.nState*self.nSegments]) +
                            [s_range]*len(self.X0[self.nState*self.nSegments:
                            (self.nState+self.len_s)*self.nSegments]) +
                        [u_range]*len(self.X0[(self.nState+self.len_s)*self.nSegments:]))

        n       = len(self.X0)
        nF      = int(1 + len(self.X0))

        F_init = [0.]*nF
        Fstate_init = [0]*nF
        constraintRelax = 0.0

        ## Setting the initial values of mu, cov and wts as boundary constraints
        Flow    = np.array([0.] + muInit.tolist() + [-constraintRelax]*len(self.X0[self.nState:self.nState*self.nSegments]) + covInit[self.id1].tolist() + [-constraintRelax]*len(self.X0[self.nState*self.nSegments + self.len_s:]))
        Fupp    = np.array([0.] + muInit.tolist() + [constraintRelax]*len(self.X0[self.nState:self.nState*self.nSegments]) + covInit[self.id1].tolist() + [constraintRelax]*len(self.X0[self.nState*self.nSegments + self.len_s:]))

        ObjRow  = 1

        Start   = 0 # Cold Start
        cw      = [None]*5000
        iw      = [None]*5000
        rw      = [None]*5000

        for trial in range(self.MAX_TRIES):
            res = snopta(self.snopt_objFun,n,nF,x0=self.X0, xlow=Xlow,xupp=Xupp, Flow=Flow,Fupp=Fupp, ObjRow=ObjRow, F=F_init, Fstate=Fstate_init, name='ds_goal', start=Start, options=options)

            if res is None:
                print "Failed to find solution for optimization after ", trial+1, " tries. Retrying!"
                continue
            else:
                # print "SNOPT Result =", np.round(res.x, 4)
                xfinal = res.x

                mu_new = np.reshape(xfinal[:self.nState*self.nSegments],(self.nState,self.nSegments),'F')
                s_new = np.reshape(xfinal[self.nState*self.nSegments:(self.nState + self.len_s)*self.nSegments], (self.len_s, self.nSegments),'F')
                u_new = np.reshape(xfinal[(self.nState+self.len_s)*self.nSegments:(self.nState+self.len_s + self.nInput)*self.nSegments],(self.nInput,self.nSegments-1),'F')

                final_wts = np.ndarray(self.nModel)
                covFinal = copy.copy(covInit)
                covFinal[self.id1] = s_new[:,-1]
                self.sysDynamics.fastWtsMapped(mu_new[:,-1], covFinal, final_wts)

                # if self.do_verbose:
                print '*****************\nSet Goal: ', goal
                print 'Plan Time Horizon: ', tFinal
                print 'Planning for segments: ', self.nSegments
                print 'Each Segment Length: ', self.delta
                print "Generated Plan: \n", np.round(mu_new.T, 3) #[-1,:]
                print "s_new: ", np.round(s_new.T, 3)
                print "u_new: ", np.round(u_new.T, 2)
                print "final_wts: ", final_wts
                print "Final Cost = ", res.F[0]
                print "********************\n"

                return res.F[0], mu_new, s_new, u_new, final_wts

        print "\nXXXXXXXXXXXXXXX FAILED TO OPTIMIZE! XXXXXXXXXXXXXXX \nReturning Initial Guess\n"
        # return np.inf, np.tile(muInit, (self.nSegments, 1)).T, np.tile(covInit[self.id1], (self.nSegments, 1)).T, np.tile([0.]*self.nInput, (self.nSegments, 1)).T, wtsInit*1.

        mu_new = np.reshape(self.X0[:self.nState*self.nSegments],(self.nState,self.nSegments),'F')
        s_new = np.reshape(self.X0[self.nState*self.nSegments:(self.nState + self.len_s)*self.nSegments], (self.len_s, self.nSegments),'F')
        u_new = np.reshape(self.X0[(self.nState+self.len_s)*self.nSegments:],(self.nInput,self.nSegments),'F')
        
        final_wts = np.ndarray(self.nModel)
        covFinal = copy.copy(covInit)
        covFinal[self.id1] = s_new[:,-1]
        self.sysDynamics.fastWtsMapped(mu_new[:,-1], covFinal, final_wts)

        return np.inf, mu_new, s_new, u_new, final_wts
