#!/usr/bin/env python

import numpy as np
import copy
from planner_interface import planner_interface

if __name__ == "__main__":
    ## Setup initial Values in the planner
    sim = True
    scale = 100.
    goal = np.array([0.95, 0.35, 0.45, -2.22])*scale

    if sim:
        x = np.array([0.92, -0.14, 0.30, -1.12])*scale
        mu = np.array([0.92, -0.14, 0.30, -1.12])*scale
        planner = planner_interface(goal, sim, x, mu)
    else:
        planner = planner_interface(goal, sim)

    mu = copy.copy(planner.mu_actual)
    x = copy.copy(planner.x_actual)

    max_threshold = 0.02
    max_final_error = 10.
    replanning_threshld = 50.
    planner.planner.nSegments = 10
    planner.t = 20
    planner.planner.dyna.goal_threshold = max_threshold

    ## Data Collection for Post-Processing and Plots
    traj = [planner.mu_actual]
    traj_true = [planner.x_actual]
    cov_traj = [planner.cov_actual[planner.id1]]
    ds_traj = [planner.wts_actual]
    
    mu_plan, s_plan, u_plan = planner.generate_plan()

    print "Planned Path: \n", np.round(mu_plan, 3)