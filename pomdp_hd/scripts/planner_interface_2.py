#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import pickle as pkl
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_msgs
from geometry_msgs.msg import Pose, PoseStamped, \
    TransformStamped, Quaternion, Vector3
from hlpr_manipulation_utils.arm_moveit2 import *
from hlpr_manipulation_utils.manipulator import Gripper


from pomdp_hd_planner import *
from pomdp_hd.src.blqr import *
from pomdp_hd.src.py_utils import arrayToPoseStamped, \
    PoseStamedToTransformStamped, invertTransformStamped, \
    PoseStampedToArray, TransformStampedToPoseStamped, \
    multiplyTransforms
from experiments.srv import *
from pomdp_hd.msg import DataPoint
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

## Utils
def multi_linspace(a, b, num=10):
    samples = []
    for i in range(len(a)):
        arr = np.linspace(a[i], b[i], num=num)
        samples.append(arr)

    pts = []
    for i in range(num):
        op = [np.round(samples[j][i], 4) for j in range(len(a))]
        pts.append(op)

    return pts

def generate_detailed_plan(plan, num=10):
    full_plan = []
    for i in range(len(plan)-1):
        full_plan += multi_linspace(plan[i], plan[i+1], num=num)

    return full_plan


def store_plan(plan, outFile="stapler_run_001"):
    a = {"mu_plan" : plan[0], "s_plan" : plan[1], "u_plan" : plan[2]}
    outFile = "../traj_data/"+ outFile + ".pkl"

    with open(outFile, 'wb') as handle:
        pkl.dump(a, handle)


class planner_interface:
    """
    Class providing a high-level interface to interact with POMDP-HD planner 
    """

    def __init__(self, goal, sim, x=None, mu=None):
        # Global Parameters
        self.SIMULATION = sim

        # Planner
        self.planner = pomdp_hd()
        
        if not self.SIMULATION:
            # Subscribers
            self.ref_pose_sub = rospy.Subscriber(
                "/changepoint/reference_frame", PoseStamped,
                self.reference_frame_cb, queue_size=1)
            self.pose_sub = rospy.Subscriber(
                "/changepoint/pose_data", PoseStamped,
                self.pose_cb, queue_size=1)
            self.obs_sub = rospy.Subscriber("/changepoint/observation_data", DataPoint, self.obs_cb)

            self.object_pose = None
            self.reference_frame = None
            self.reference_frame_fixed = None

            # Arm Motion
            self.arm = ArmMoveIt(
                planning_frame='linear_actuator_link', _arm_name='right')
                # planning_frame='linear_actuator_link', _arm_name='left')

            self.gripper = Gripper()

            ## Add Collision Object
            self.collision_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=1, latch=True)

            # self.publish_collision_object()
            # raw_input('Check if collision object available')

            # self.set_up_experiment()

            ## Go to Home Pose
            # start_pose = [4.273, 2.483, 6.336, 3.028, 0.265, 5.1514, 3.514]
            # start_pose = [4.3025, 2.4221, 0.36277, 2.6103, 5.5443, 5.0033, 3.15467]
            # self.arm.move_to_joint_pose(start_pose)


        ''' Set Planner Parmeters'''
        self.planner.do_parallelize = True
        self.planner.do_verbose = False
        # Number fo Samples to calculate discerte belief from continous beleif
        self.planner.dyna.nSamples = 100.
        # Change the loop count in beleif propagation int value
        self.planner.dyna.ds_res_loop_count_ = 1
        # Scaling factor for easier optimization
        self.scale = 100.
        # self.scale = 1.

        ''' Initialization '''
        # self.id1 = self.planner.opt.id1
        self.id1 = np.triu_indices(self.planner.nState)
        self.idx = 0

        rospy.sleep(2)

        if self.SIMULATION:
            self.x_actual = copy.copy(x)
            self.mu_actual = copy.copy(mu)
        else:
            self.check_if_data_available()
            self.mu_actual = self.get_observation()
            self.x_actual = copy.copy(self.mu_actual)

        self.cov_actual = (np.linalg.norm(self.x_actual-self.mu_actual)**2
                           + 10.) * np.eye(self.planner.nState)
        # self.cov_actual = (np.linalg.norm(self.x_actual-self.mu_actual)**2
                           # + 0.02) * np.eye(self.planner.nState)

        self.s_actual = copy.copy(self.cov_actual[self.id1])
        self.wts_actual = np.ndarray(self.planner.nModel)
        self.goal = copy.copy(goal)

        self.planner.start_mu = copy.copy(self.mu_actual)
        self.planner.start_cov = copy.copy(self.cov_actual)
        self.planner.belief = hybrid_belief(
            self.mu_actual, self.cov_actual, self.wts_actual)
        self.planner.goal_cs = copy.copy(self.goal)
        self.planner.dyna.setGoalGC(copy.copy(self.goal))  # Setting GC

        # Data
        self.idx_old = 0
        self.cov_plan = symarray(
            np.zeros((self.planner.nState, self.planner.nState)))
        self.xNew = np.ndarray(self.planner.nState)
        self.z = np.ndarray(self.planner.nState)
        self.muNew = np.ndarray(self.planner.nState)
        self.covNew = np.ndarray((self.planner.nState, self.planner.nState))
        self.wtsNew = np.ndarray(self.planner.nModel)

        # BLQR
        Q = 0.5*np.eye(self.planner.nState)
        R = 0.5*np.eye(self.planner.nInput)
        Q_f = 0.5*np.eye(self.planner.nState)
        labda = 0.
        # self.controller = blqr(self.planner.nState, self.planner.nInput,
        #                        self.planner.nOutput, self.planner.opt.Q,
        #                        self.planner.opt.R, self.planner.opt.labda,
        #                        self.planner.opt.Q_f)
        self.controller = blqr(self.planner.nState, self.planner.nInput,
                               self.planner.nOutput, Q, R, labda, Q_f)

        # Matrices for holding dynamics
        self.A = np.ndarray((self.planner.nState, self.planner.nState))
        self.B = np.ndarray((self.planner.nState, self.planner.nInput))
        self.C = np.ndarray((self.planner.nOutput, self.planner.nOutput))
        self.V = np.ndarray((self.planner.nState, self.planner.nState))
        self.W = np.ndarray((self.planner.nOutput, self.planner.nOutput))

        # Experimental Data Storage
        self.traj = [copy.copy(self.mu_actual)]
        self.traj_true = [copy.copy(self.x_actual)]
        self.ds_traj = [copy.copy(self.wts_actual)]
        self.cov_traj = [copy.copy(self.s_actual)]

        # Debug Utils
        if not self.SIMULATION:
            rospy.wait_for_service('/broadcast_frame')
            self.tf_broadcaster = rospy.ServiceProxy(
                '/broadcast_frame', frame_data)
            rospy.loginfo('broadcast_frame service: Available!')
            self.display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Checking if everything is correctly set
        print "start_mu: ", self.planner.start_mu
        print "start_cov:", self.planner.start_cov, "\ngoal: ", self.goal

        rospy.sleep(2)


    def check_if_data_available(self):
        while self.object_pose is None and \
                self.reference_frame is None:
            rospy.loginfo("Waiting for data topics to start!")
            rospy.sleep(1)
            self.reference_frame_fixed = copy.copy(
                self.reference_frame)
        rospy.loginfo('Date Topics: Available!')

    # Callbacks

    def reference_frame_cb(self, msg):
        self.reference_frame = PoseStamedToTransformStamped(
            msg, child_frame='stapler_lower_arm')
        if self.reference_frame_fixed is None:
            self.reference_frame_fixed = copy.copy(self.reference_frame)

        """For Stapler"""
        arr = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w ])
        self.planner.dyna.setReferenceFrameMapped(arr)
        # print "Transform: ", arr

    def pose_cb(self, msg):
        self.object_pose = msg

    # Utilities
    def check_if_plan_safe(self, plan):
        cur_jnt_pose = np.array(self.arm.get_current_pose(group_id=0))

        plan_ok = False
        if len(plan.joint_trajectory.points) > 0:
            for pt in plan.joint_trajectory.points:
                new_jnt_pose = np.array(pt.positions)
                if np.amax(new_jnt_pose - cur_jnt_pose) > 0.25:
                    print "Max_val: ", np.amax(new_jnt_pose - cur_jnt_pose)
                    plan_ok = False
                    break
                else:
                    plan_ok = True
                    cur_jnt_pose = copy.copy(new_jnt_pose)

        return plan_ok

    def gen_coord_to_ee_target(self, q):
        q = np.array(q)
        target_set_pt_array = np.ndarray(7)


        """ DRAWER """
        # self.planner.dyna.forwardKinematicsMapped(
        #     self.idx, q, target_set_pt_array)

        # target_set_pt = arrayToPoseStamped(
        #     target_set_pt_array, "chips_can")

        # print "target_set_pt wrt Chips Can: ", target_set_pt


        """ Stapler"""
        cov = 1e-6*np.eye(self.nState)
        wts = np.ndarray(self.planner.nModel)
        self.planner.dyna.fastWtsMapped(q, cov, wts)
        idx = self.planner.wts2ds(wts)
        self.planner.dyna.forwardKinematicsMapped(idx, q, target_set_pt_array)

        target_set_pt = arrayToPoseStamped(
            target_set_pt_array, "stapler_lower_arm")

        print "target_set_pt wrt stapler_lower_arm: ", target_set_pt

        # Convert pose to linear_actuator_frame
        transform_to_LA_frame = TransformStamped()
        transform_to_LA_frame = copy.copy(self.reference_frame)
        # transform_to_LA_frame = copy.copy(self.reference_frame_fixed)

        # # Fix orientation
        # transform_to_LA_frame.transform.rotation = \
        #     Quaternion(0.707, 0., -0.707, 0.)

        #####################
        ### TASK SPECIFIC ###
        #####################
        # Task-specific Offset
        # offset_tf.header = transform_to_LA_frame.header

        '''
        ## MICROWAVE        
        offset_tf.transform.translation = Vector3(-0.1, -0.21, -0.32)
        offset_tf.transform.rotation = Quaternion(-0.174, 0., 0., 0.985)
        '''

        ## DRAWER
        offset_tf = TransformStamped()
        offset_tf.header.stamp = target_set_pt.header.stamp
        offset_tf.header.frame_id = "drawer"

        offset_tf.transform.translation = Vector3(-0.070, 0.125, 0.017)
        offset_tf.transform.rotation = Quaternion(-0.653, 0.757, -0.004, -0.003)
        object_to_ref_frame = PoseStamedToTransformStamped(target_set_pt)
        eef_in_ref_frame = multiplyTransforms(object_to_ref_frame, offset_tf)
        eef_target = TransformStampedToPoseStamped(eef_in_ref_frame)
        # final_tf = multiplyTransforms(offset_tf, transform_to_LA_frame)
        # final_tf = transform_to_LA_frame

        # target_ee_pose2 = tf2_geometry_msgs.do_transform_pose(
            # target_set_pt, transform_to_LA_frame)

        target_ee_pose2 = tf2_geometry_msgs.do_transform_pose(
            eef_target, transform_to_LA_frame)


        target_ee_pose2.pose.orientation = Quaternion(0., 0., 0., 1.)

        return target_ee_pose2


    def broadcast_plan_in_cart_space(self, plan):
        poses = []
        parent_frames = []
        child_frames = []

        # plan = plan.T

        # Convert a plan to pose in cart space
        for i, q in enumerate(plan):
            tar_pose = self.gen_coord_to_ee_target(q)
            print "Target pt: ", q
            print "tar_pose: ", tar_pose
            poses.append(tar_pose)
            child_frames.append('t_frame_' + str(i))
            parent_frames.append('linear_actuator_link')

        # Transforming Frame
        # transform_to_LA_frame = TransformStamped()
        # transform_to_LA_frame = copy.copy(self.reference_frame)
        # # Fix orientation
        # transform_to_LA_frame.transform.rotation = \
        #     Quaternion(0.707, 0., -0.707, 0.)
        # poses.append(TransformStampedToPoseStamped(transform_to_LA_frame))
        # child_frames.append('transforming_frame')
        # parent_frames.append('linear_actuator_link')

        # Center Frame
        # tar_pose1 = self.gen_coord_to_ee_target([0.])
        # tar_pose2 = self.gen_coord_to_ee_target([np.pi*self.scale])
        # tar_pose = copy.copy(tar_pose1)
        # tar_pose.pose.position.x = (
        #     tar_pose1.pose.position.x + tar_pose2.pose.position.x) / 2
        # tar_pose.pose.position.y = (
        #     tar_pose1.pose.position.y + tar_pose2.pose.position.y) / 2
        # tar_pose.pose.position.z = (
        #     tar_pose1.pose.position.z + tar_pose2.pose.position.z)/ 2
        # poses.append(tar_pose)
        # child_frames.append('center')
        # parent_frames.append('linear_actuator_link')
        # print "Center_pose: ", tar_pose

        # Make a request to publish it on the tf tree
        req = frame_dataRequest()
        req.parent_frames = parent_frames
        req.child_frames = child_frames
        req.poses = poses
        req.reset = True
        self.tf_broadcaster(req)

        self.planned_poses = copy.copy(poses)

    # UI Functions

    def generate_plan(self):
        """
        Function to generate plan based on set continuous space target

        Returns
        -------
            mu_plan : planned belief mean trajectory
            s_plan  : Covariance vector based on planned trajectory
            u_plan  : Planned control to be applied 
        """
        start_time = time.time()

        '''For MICROWAVE and DRAWER'''
        # self.planner.direct_planning = True
        # self.planner.opt.nSegments = copy.copy(self.planner.nSegments)
        # mu_plan, s_plan, u_plan = self.planner.find_optimal_path()

        '''For STAPLER'''
        mu_plan, s_plan, u_plan = self.planner.plan_optimal_path()

        # mu_plan = [self.mu_actual]
        # s_plan = [self.s_actual]
        # u_plan = [np.array([0.]*3)]


        # cost, mu_plan, s_plan, u_plan, final_wts = \
        #     self.planner.opt.cs_optimize(
        #         self.planner.start_mu, self.planner.start_cov,
        #         self.planner.belief.wts, self.planner.t,  self.planner.goal_cs)
         
        # planned_path = [[mu_plan], [s_plan], [u_plan]]

        ## Can be used to generate smoother trajectories
        # mu_plan, s_plan, u_plan = self.planner.generateDetailedCSPlan(planned_path)

        tActual = np.round((time.time() - start_time), 3)
        print("Total time required by Planner = %s seconds " % tActual)


        detailed_plan = generate_detailed_plan(mu_plan.T, num=5)
        print "Detailed plan: ", detailed_plan
        
        # self.planner.plot_trajectory(detailed_plan)
        f_name = "stapler_run_"+str(np.random.randint(0, 1000))
        store_plan([mu_plan, s_plan, u_plan], outFile=f_name)
        print " Stored in file: ", f_name

        if not self.SIMULATION:
            self.broadcast_plan_in_cart_space(mu_plan.T)
            # self.broadcast_plan_in_cart_space(detailed_plan)
            # self.execute_complete_plan(self.planned_poses)
            self.execute_plan_in_steps()
            # self.execute_complete_plan(detailed_plan)

        return mu_plan, s_plan, u_plan

    def convert_plan_to_cart_traj(self, planned_waypts):
        waypoints = []       
        # waypoints.append(self.arm.get_FK(root="linear_actuator_link")[0].pose)

        for pt in planned_waypts:
            waypoints.append(copy.copy(pt.pose))

        (traj, fraction) = self.arm.group[0].compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold

        print "Trajectory success fraction: ", fraction

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = \
                self.arm.robot.get_current_state()
        display_trajectory.trajectory.append(traj)
        self.display_trajectory_publisher.publish(display_trajectory)

        return traj

        # waypoints = []

        # for pt in planned_waypts:
        #     waypoints.append(pt.pose)

        # plans = self.arm.plan_waypoints(waypoints, is_joint_pos=False)

        # # Display planned traj
        # for traj in plans:
        #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #     # display_trajectory.trajectory_start = \
        #         # self.arm.robot.get_current_state()
        #     display_trajectory.trajectory_start = \
        #         traj.joint_trajectory.points[0]

        #     display_trajectory.trajectory.append(traj)
        #     self.display_trajectory_publisher.publish(display_trajectory)

        # return plans

    def execute_complete_plan(self, plan):
        #### Execute Arm Motion
        plan_safe = False
        while not plan_safe:
            plan = self.convert_plan_to_cart_traj(self.planned_poses)
            plan_safe = self.check_if_plan_safe(plan)
            print "Plan_safe : ", plan_safe     

        raw_input("Press Enter to move arm!")
        res = self.arm.move_robot(plan)
        res =True
            
        # plans = self.convert_plan_to_cart_traj(self.planned_poses)
        # for plan in plans:
        #     raw_input('Press Enter to move arm!')
        #     res = self.arm.move_robot(plan)

        return res


    def execute_plan_in_steps(self):
        raw_input("Press Enter to move arm!")

        for pos in self.planned_poses:
            print "Going to pose: ", pos
            self.arm.move_to_ee_pose(pos.pose)
            # raw_input("Press Enter to move arm!")
            rospy.sleep(2)

        res = True
        return res



    def execute_plan_oneStep(self, mu_plan, s_plan, u_plan):
        """
        Function to execute plan one step based on applied controls.
        First it calculates the local belief space control (B-LQR) 
        that it should apply based on the current belief and covariance.
        Executes one step using simulator and returns new robot state 
        and observation

        Parameters
        -------
            mu_plan : Planned next step belief mean
            s_plan  : Planned next step belief covariance
            u_plan  : Planned control input


        Returns
        -------
            xNew    : New robot state from simulator
            z       : Observation from simulator

        """
        self.cov_plan[self.id1] = copy.copy(s_plan)

        mu_plan = np.array(mu_plan)
        u_plan = np.array(u_plan)

        # B-LQR Control
        self.planner.dyna.getMatrices(
            self.idx, mu_plan, u_plan, self.A, self.B, self.C, self.V, self.W)

        u_local = self.controller.blqr(
            self.mu_actual, self.s_actual, mu_plan,
            s_plan, u_plan, self.A, self.B, self.C, self.W, 5)

        # Propagation of the Belief continuous Dynamics
        self.idx = self.planner.dyna.predictionStochastic(
            self.mu_actual, self.cov_actual, u_local,
            self.muNew, self.covNew, self.wtsNew, self.idx)

        # Execution
        z = np.ndarray(self.planner.nOutput)

        if self.SIMULATION:
            print "Executing in SIMULATION!"
            self.planner.dyna.simulateOneStep(
                self.x_actual, u_local, self.xNew, z)
        else:
            print "Executing on REAL ARM!"

            tarPose = self.gen_coord_to_ee_target(self.muNew)
            print "tarPose:", tarPose

            plan_safe = False
            while not plan_safe:
                plan = self.arm.plan_pose(tarPose.pose, is_joint_pos=False)
                plan_safe = self.check_if_plan_safe(plan)
                print "Plan_safe : ", plan_safe

            raw_input("Press Enter to move arm!")
            res = self.arm.move_robot(plan)

            # Generate Observation
            rospy.sleep(0.1)
            cur_pose = PoseStampedToArray(self.object_pose, euler=False)
            self.planner.dyna.inverseKinematicsMapped(
                self.idx, cur_pose, z)

        return z


    def obs_cb(self, msg):
        self.obs = np.array([msg.point[0],msg.point[1], msg.point[2], msg.point[3]])

    def get_observation(self):
        z = np.ndarray(self.planner.nOutput)
        cur_pose = PoseStampedToArray(self.object_pose, euler=False)
        self.planner.dyna.inverseKinematicsMapped(
            self.idx, cur_pose, z)
        return z
        # return self.obs*self.scale

    def update_belief(self, z):
        """
        Update belief based on received observation

        Parameters
        -------
            z       : Recievd Observation

        """
        # Updates based on the observation
        self.idx = self.planner.dyna.observationUpdate(
            z, self.muNew, self.covNew, self.wtsNew, self.idx)
        return

    def update_stored_values(self):
        """
        Updates values in the planner
        """
        # Update belief Values
        self.mu_actual = copy.copy(self.muNew.T)
        self.planner.start_mu = copy.copy(self.mu_actual)
        self.x_actual = copy.copy(self.xNew)
        self.cov_actual = copy.copy(self.covNew)
        self.planner.start_cov = copy.copy(self.cov_actual)
        self.s_actual = copy.copy(self.cov_actual[self.id1])
        self.wts_actual = copy.copy(self.wtsNew)


    def remove_collosion_object(self):
        msg = CollisionObject()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = "linear_actuator_link"
        msg.id = "table"  
        msg.operation = msg.REMOVE
        self.collision_pub.publish(msg)


    def publish_collision_object(self):
        self.remove_collosion_object()

        table_pose = Pose()
        table_pose.position.x = 0.64 + 0.76/2
        table_pose.position.y = 0.
        table_pose.position.z = -1.15 + 1.05/2
        table_pose.orientation.x = 0.
        table_pose.orientation.y = 0.
        table_pose.orientation.z = 0.
        table_pose.orientation.w = 1.
        # scene.add_box('cabinet', table_pose, (0.05, 0.05, 1.05))

        shape = SolidPrimitive()
        shape.type = shape.BOX
        shape.dimensions = [0.76, 1.6, 1.05]
        # shape.dimensions = [0.05, 0.05, 1.05]


        msg = CollisionObject()
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = "linear_actuator_link"
        msg.id = "table"
        msg.primitives = [shape]
        msg.primitive_poses =  [table_pose]
        msg.operation = msg.ADD
        self.collision_pub.publish(msg)


    def set_up_experiment(self):
        ## Go to pick up stapler
        start_jnt_pose = [3.726, 2.095, -0.5558755889302731, 1.8383934162871485, 3.7721823280853712, 1.6595971696566623, 5.88241471684905]
        self.arm.move_to_joint_pose(start_jnt_pose)

        pickup_jnt_pose = [3.501, 2.220, -0.101, 1.614, 3.887, 1.953, 5.883]
        self.arm.move_to_joint_pose(pickup_jnt_pose)
        rospy.sleep(1)

        ## Open gripper
        self.gripper.open()

        ## Grasp gripper
        self.gripper.close(force=150)
        rospy.sleep(1)

        ## Go to home pose
        self.arm.move_to_joint_pose(start_jnt_pose)
