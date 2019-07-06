#!/usr/bin/env python

import signal
import sys

import numpy as np
import copy
import rospy

from planner_interface import planner_interface

def signal_handler(sig, frame):
        print 'Ctrl+C Pressed! Killing the node'
        rospy.signal_shutdown('You pressed Ctrl+C!') 
        sys.exit(0)


if __name__ == "__main__":
    rospy.init_node("pomdp_executer_node", anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)

    ## Setup initial Values in the planner
    sim = True
    goal = np.array([0.85, 0.35, 0.45, 0.])*100.

    # goal = np.array([0.721, -0.028, -0.197, 2.57])

    if sim:
        x = np.array([81., -14., 30., 157.])
        mu = np.array([81., -14., 30., 157.])
        planner = planner_interface(goal, sim, x, mu)
    else:
        planner = planner_interface(goal, sim)

    mu = copy.copy(planner.mu_actual)
    x = copy.copy(planner.x_actual)

    max_threshold = 0.02
    max_final_error = 10.
    replanning_threshld = 50.
    planner.planner.nSegments = 4
    planner.t = 16
    planner.planner.dyna.goal_threshold = max_threshold

    ## Data Collection for Post-Processing and Plots
    traj = [planner.mu_actual]
    traj_true = [planner.x_actual]
    cov_traj = [planner.cov_actual[planner.id1]]
    ds_traj = [planner.wts_actual]

    # rospy.sleep(3)
    
    mu_plan, s_plan, u_plan = planner.generate_plan()

    print "Planned Path: \n", np.round(mu_plan, 3)

    # rospy.sleep(2)
    # goal2 = np.array([5.])
    # planner2 = planner_interface(goal2, sim)
    # mu_plan, s_plan, u_plan = planner2.generate_plan()

    # while((max(abs(mu - goal)) > max_final_error)):

        ## Generate Planned Trajectory
        # mu_plan, s_plan, u_plan = planner.generate_plan()

    #     ## Execute Planner Trajectory step-by-step
    #     for t in range(len(mu_plan.T)-1):
    #         z = planner.execute_plan_oneStep(mu_plan[:,t], s_plan[:,t], u_plan[:,t])
    #         planner.update_belief(z)

    # #         print "\ntime Step =", t + 1
    # #         print "Active Model id =",  planner.idx
    # #         print "Observation = ", z
    # #         print "mu after EKF = ", np.round(planner.muNew, 3)
    # #         print "actual x = ", np.round(x, 3)
    # #         print "planned s at t+1 =", s_plan[:, t]
    # #         print "Expected actual s at t + 1=", planner.covNew[planner.id1]
    # #         print "Current mu from optimization = ", np.round(mu_plan[:,t], 3)
    # #         print "Next Step Planned mu from optimization = ", np.round(mu_plan[:,t+1], 3)
    # #         print "Model wts =", planner.wtsNew

    #         ## Updates for Next iteration
    #         planner.update_stored_values()
    #         mu = copy.copy(planner.mu_actual)
    #         x = copy.copy(planner.x_actual)
    #         traj.append(np.round(mu, 3))
    #         traj_true.append(np.round(x, 3))
    #         cov_traj.append(s_plan[:, t+1])
    #         ds_traj.append(planner.wts_actual)

    #         ### Replan Trajecotry if belief diverged too far.
    #         if(max(abs(mu-mu_plan[:,t+1])) > replanning_threshld):
    #             print "BLQR unable to stabilize. Replanning \n"
    #             raw_input('press Enter to replan')
    #             break

    #     print "\n####################"
    #     print "mu = ", np.round(mu, 3)
    #     print "x = ", np.round(x, 3)
    #     print "#####################\n"
        
    # Plotting
    # import matplotlib.pyplot as plt
    # fig = plt.figure(1)
    # x_vec = [traj[i][0] for i in range(len(traj))]
    # x_vec_true = [traj_true[i][0] for i in range(len(traj_true))]
    # t_vec = range(len(traj))
    # bl1, = plt.plot(t_vec, x_vec, "b")
    # bl2, = plt.plot(t_vec, x_vec_true, "k")
    # plt.title('Belief and Actual Trajectories')
    # plt.legend([bl1, bl2], ['Belief Trajectory', 'Actual Trajectory'])
    # fig.show()
    # raw_input('Press enter to close')

    print "REACHED GOAL. TASK COMPLETED!"
    # rospy.spin()