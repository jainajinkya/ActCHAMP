#!/usr/bin/env python

import rospy
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Gripper
from geometry_msgs.msg import Pose
import moveit_msgs

jnt_traj = [[0.4402,  3.73,  3.3201,  0.9605,  3.3041,  4.6171,  8.8072],
            [0.4402,  3.73,  3.3134,  1.0521,  3.3038,  4.5272,  8.8046],
            [0.4442,  3.6612,  3.2668,  1.0686,  3.2674,  4.5671,  8.8415],
            [0.4442,  3.6579,  3.261,  1.1636,  3.2617,  4.4801,  8.8414],
            [0.4442,  3.6487,  3.2569,  1.2617,  3.2596,  4.3966,  8.8387],
            [0.4408,  3.7103,  3.2981,  1.2472,  3.2915,  4.3586,  8.7999],
            [0.4384,  3.6937,  3.297,  1.3496,  3.2915,  4.2826,  8.7924],
            [0.4384,  3.7103,  3.298,  1.2483,  3.2915,  4.3564,  8.7961],
            [0.4287,  3.7679,  3.3408,  1.2397,  3.3323,  4.3181,  8.7627],
            [0.4159,  3.8249,  3.3817,  1.236,  3.375,  4.2774,  8.7271],
            [0.414,  3.802,  3.3775,  1.3373,  3.3749,  4.2085,  8.7175],
            [0.4288,  3.7463,  3.3378,  1.34,  3.332,  4.2481,  8.7514],
            [0.4307,  3.766,  3.3407,  1.2374,  3.332,  4.3206,  8.7608],
            [0.4405,  3.707,  3.2997,  1.2457,  3.2952,  4.3587,  8.7952],
            [0.4405,  3.6899,  3.2971,  1.3484,  3.2952,  4.2814,  8.7911],
            [0.465,  3.7194,  3.2064,  1.3398,  3.2444,  4.2559,  8.8545]]


side_traj = [[0.6949,  4.2381,  3.9862,  1.1, -2.2317,  4.3514,  7.8967],
             [0.5729,  4.1517,  4.0721,  1.0894, -2.2655,  4.4467,  7.9992],
             [0.5828,  4.1105,  4.0255,  1.1759, -2.3103,  4.3911,  7.961],
             [0.5828,  4.072,  3.9925,  1.2656, -2.3335,  4.3363,  7.9296],
             [0.4983,  4.0357,  4.0597,  1.2956, -2.3318,  4.3733,  7.9527],
             [0.4947,  4.0744,  4.0949,  1.2049, -2.3011,  4.4338,  7.987],
             [0.4853,  4.1129,  4.1374,  1.1182, -2.2653,  4.4921,  8.0243],
             [0.5468,  4.1323,  4.0973,  1.0945, -2.2711,  4.472,  8.0049],
             [0.5114,  4.1195,  4.1221,  1.108, -2.2655,  4.485,  8.0147],
             [0.5115,  4.1055,  4.1071,  1.0866, -2.2878,  4.4959,  8.0407],
             [0.5154,  4.0923,  4.09,  1.1175, -2.3012,  4.4742,  8.0283],
             [0.523,  4.0539,  4.0475,  1.205, -2.3393,  4.4155,  7.9941]]

top_traj = [[0.4854,  3.7798,  3.2506,  1.0108, -2.9675,  4.4943,  8.737],
            [0.4848,  3.7724,  3.2474,  1.0946, -2.9675,  4.4238,  8.7295],
            [0.4843,  3.762,  3.2437,  1.1832, -2.9675,  4.3509,  8.7201],
            [0.4707,  3.8178,  3.2872,  1.1764, -2.9302,  4.3108,  8.6865],
            [0.4707,  3.8347,  3.2922,  1.0893, -2.9302,  4.3731,  8.6927],
            [0.4573,  3.8922,  3.3336,  1.0866, -2.889,  4.3298,  8.6616],
            [0.4554,  3.8748,  3.3281,  1.1717, -2.8891,  4.2696,  8.652],
            [0.4711,  3.8209,  3.2865,  1.1732, -2.9311,  4.3111,  8.6849],
            [0.477,  3.8242,  3.2708,  1.1731, -2.9429,  4.3087,  8.6926],
            [0.4782,  3.8365,  3.2709,  1.1185, -2.9451,  4.3481,  8.6987],
            [0.498,  3.8552,  3.2275,  1.1145, -2.976,  4.3324,  8.7224],
            [0.4766,  3.8344,  3.282,  1.1167, -2.9412,  4.354,  8.6987],
            [0.462,  3.8206,  3.3177,  1.1215, -2.9175,  4.3684,  8.6807],
            [0.462,  3.8206,  3.3177,  1.1215, -2.9175,  4.3684,  8.6807],
            [0.4688,  3.8356,  3.3069,  1.0385, -2.926,  4.4326,  8.6919]]

stapler_traj = [[4.454221345364082, 2.8703286575926112, -2.722170081932669, 2.5190433287412732, 7.090323387187569, 4.349509313411583, 5.471614661853275,],
                [4.4544458498439745, 2.8626417094222063, -2.721481654790883, 2.4802346800722024, 7.151441868940451, 4.277571473414144, 5.471614661853275],
                [4.461506156209884, 2.844148718812696, -2.6722169030499434, 2.452524755246037, 7.153683185313863, 4.246941392452707, 5.471614661853275],
                [4.462553044834394, 2.8422823755207687, -2.6272781901837288, 2.4311973622883554, 7.154674946503809, 4.215941931048076, 5.471614661853275],
                [4.481517947589067, 2.841827241290474, -2.5896051134034677, 2.4302932190982736, 7.154706904436889, 4.215798120349211, 5.471611998692185],
                [4.509837469988751, 2.8244011130136495, -2.5872173231701026, 2.376471532416099, 7.154699447585837, 4.1147822896736335, 5.471611998692185],
                [4.53157365817376, 2.7834337059649608, -2.5422170912827067, 2.396170668683306, 7.152429369072653, 4.107612527386923, 5.471584834449066],
                [4.576021284134964, 2.756573595842639, -2.5423382651123054, 2.3771612911384294, 7.144024432672364, 4.032246133802116, 5.471574714436924],
                [4.576022882031618, 2.7565821179581276, -2.542327878784054, 2.3771631553511927, 7.144024432672364, 4.032248530647097, 5.471574714436924]]

stapler_traj_2 = [[3.184135054107155, 2.459173869539638, -0.26255652449705896, 1.394529151093604, 4.965060117231483, 4.210893909201831, 3.9424781639904882],
                  [3.253759672381751, 2.4584905024039236, -0.33602737988648784, 1.421904183412692, 4.962796430304916, 4.14637643494968, 3.9424832239965593],
                  [3.3932395386813425, 2.352206140143842, -0.5388878169688756, 1.464131665131152, 5.158704951677761, 3.905270607768773, 3.7421322836093536],
                  [3.4586092244823186, 2.3531110822822514, -0.599434085272041, 1.4882262156194805, 5.110098533358289, 3.877540975550541, 3.7421346804543343],
                  [3.5250215378500602, 2.353774209393681, -0.6243680641359624, 1.4998503811454582, 5.109511040021817, 3.8736165413682, 3.7421346804543343],
                  [3.597980966441653, 2.3538714147734687, -0.6683217402939844, 1.5172354967414954, 5.035882092732179, 3.8725382274428273, 3.7421346804543343],
                  [3.597980966441653, 2.3538714147734687, -0.6683217402939844, 1.5172354967414954, 5.035879962203307, 3.8725382274428273, 3.7421346804543343]]


stapler_1 = [[0.841, -0.092, 0.102, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, 0.098, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, 0.078, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, 0.023, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, 0.0, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, -0.03, 0.664, 0.662, -0.267, 0.223],
              [0.9, -0.2, -0.08, 0.664, 0.662, -0.267, 0.223],
              [0.85, -0.2, 0., 0.664, 0.662, -0.267, 0.223],
              [0.85, -0.2, 0.05, 0.664, 0.662, -0.267, 0.223],
              [0.8, -0.20, 14.0, 0.664, 0.662, -0.267, 0.223],
              [0.77, -0.15, 0.05, 0.664, 0.662, -0.267, 0.223],
              [0.75, -0.10, 0.0, 0.664, 0.662, -0.267, 0.223],
              [0.73, -0.05, -0.10, 0.664, 0.662, -0.267, 0.223],
              [0.721, -0.028, -0.197, 0.664, 0.662, -0.267, 0.223]]

stapler_cart_traj = [[  80.88 ,   -5.146,   20.133,  174.404],
                     [  81.742,    2.2  ,   22.895,  160.   ],
                     [  80.86 ,    2.2  ,   21.578,  160.   ],
                     [  80.364,    2.2  ,   18.057,  160.   ],
                     [  80.623,    2.2  ,    8.029,  160.   ],
                     [  78.502,    2.2  ,   -0.731,  157.08 ],
                     [  78.689,    2.2  ,   -0.731,   74.062],
                     [  80.224,    2.2  ,   -0.731,   74.062],
                     [  80.224,    2.2  ,   5.3,   74.062],
                     [  80.106,    2.2  ,    9.904,    0.   ],
                     [  80.609,    2.2  ,   10.086,    0.   ],
                     [  80.973,    2.2  ,   21.319,    0.   ],
                     [  80.588,    2.2  ,   22.076,    0.   ],
                     [  80.081,    2.2  ,   24.864,    0.   ],
                     [  80.404,    2.2  ,   27.279,    0.   ],
                     [  80.531,    2.2  ,   27.935,    0.   ]]


stapler_cart_traj_2 = [[  88.661,  -11.383,   40.241,  181.128],
                       [  81.908,   -7.441,   20.333,  160.   ],
                       [  83.452,   -7.441,   18.167,  160.   ],
                       [  81.383,   -7.441,   16.387,  160.   ],
                       [  79.799,   -7.441,    9.263,  160.   ],
                       [  79.884,    9.22 ,   11.625,  135.872],
                       [  79.853,    9.22 ,    4.841,  160.   ],
                       [  78.26 ,    9.22 ,    5.246,  155.569],
                       [  77.459,    9.22 ,   -2.709,  157.08 ],
                       [  78.397,    9.22 ,   -2.709,   92.634],
                       [  74.854,    9.22 ,   -2.709,   58.695],
                       [  75.108,    9.22 ,   -5.709,   24.852],
                       [  81.222,    9.22 ,   -5.709,    0.295],
                       [  80.344,    9.22 ,   -5.069,    0.   ],
                       [  79.914,    9.22 ,   -5.249,    0.   ],
                       [  77.897,    9.22 ,   -5.896,    0.   ],
                       [  77.726,    9.22 ,    1.355,    0.   ],
                       [  77.96 ,    9.22 ,    4.826,    0.   ],
                       [  77.248,    9.22 ,    8.93 ,    0.   ],
                       [  75.74 ,    9.22 ,   22.119,    0.   ],
                       [  77.79 ,    9.22 ,   26.861,    0.   ],
                       [  77.719,    9.22 ,   27.65 ,    0.   ],
                       [  78.422 ,   12.22  ,  28.86  ,   0.   ],
                       [  78.422 ,   15.22  ,  30.86  ,   0.   ],
                       [  78.422 ,   15.22  ,  35.86  ,   0.   ],
                       [  78.422 ,   20.22  ,  42.86  ,   0.   ],
                       [  78.422 ,   25.22  ,  42.86  ,   0.   ],
                       [  78.422 ,   30.22  ,  44.86  ,   0.   ]]

stapler_cart_traj_fail = [[  85.943,   -7.492,   11.495,  138.551],
                         [  79.482,    8.705,    3.13 ,  160.   ],
                         [  78.358,    8.705,    3.398,  157.053],
                         [  77.784,    8.705,    3.616,  154.701],
                         [  76.02 ,    8.705,    4.002,  150.635],
                         [  76.452,    8.705,    4.534,  145.218],
                         [  76.607,    8.705,    4.883,  141.757],
                         [  74.664,    8.705,    5.493,  135.862],
                         [  75.129,    8.705,    6.196,  129.286],
                         [  78.   ,    8.705,    6.917,  122.712],
                         [  79.166 ,   8.705 ,  24.679 ,   0.   ],
                         [  79.166 ,   10.705 ,  34. ,   0.   ],
                         [  79.166 ,   15. ,  40. ,   0.   ],
                         [  79.166 ,   25. ,  40.679 ,   0.   ]]

stapler_cart_traj_3 = [[  84.00011654   , 3.02072476  , 19.06544846 , 180.52517168],
                      [  82.14408081   , 3.02072476  ,  9.06544846 , 160.        ],
                      [ 79.63913428,   3.02072476,  0.,  92.48549877],
                      [  76.33269149   , 3.02072476  ,-8.36726809 , 100.80218969],
                      [  74.82537696   , 3.02072476  ,-8.61513884 , 120.413973  ],
                      [  74.05947753   , 3.02072476  ,-8.1095306  , 157.07963268],
                      [ 76.85452045,   3.02072476, -8.1095306 ,  61.70493622],
                      [ 77.00630878,   3.02072476, -8.67791897,   0.        ],
                      [ 76.94764052,   3.02072476,  -1.7720942 ,   0.        ],
                      [ 75.88564308,   3.02072476,  15.66805795,   0.        ],
                      [ 77.16673931,   3.02072476,  18.81378472,   0.        ],
                      [ 75.80438527,   3.02072476,  18.29612899,   0.        ],
                      [ 78.70322155,   10.53005273,  23.84773765,   0.        ],
                      [ 80.97559662,   16.53005273,  32.60798696,   0.        ],
                      [ 82.02769951,   16.53005273,  35.37050209,   0.        ],
                      [ 82.02769951,   20.53005273,  40.37050209,   0.        ],
                      [ 82.02769951,   22.53005273,  45.37050209,   0.        ],
                      [ 82.02769951,   27.53005273,  45.37050209,   0.        ],
                      [ 82.02769951,   36.53005273,  45.37050209,   0.        ]]



def threshold_values(val, thres):
    if val > thres :
      return thres
    elif val< -thres :
      return -thres
    else:
      return val


if __name__ == "__main__":
    rospy.init_node('saved_trajectory_playback', anonymous=True)

    # Arm execution
    reference_frame = "linear_actuator_link"
    arm_name = "right"
    arm = ArmMoveIt(planning_frame=reference_frame,
                    _arm_name=arm_name)

    gripper = Gripper()

    z_offset = -0.1
    scale = 100
    traj_to_follow = stapler_cart_traj_3

    poses = []

    ## Open gripper
    gripper.open()
    rospy.sleep(1)

    ## Go to pick up stapler
    # start_jnt_pose = [3.726, 2.095, -0.555, 1.8383, 3.772, 1.659, 5.8824]
    start_jnt_pose = [4.136, 2.152, -0.8295, 1.7024, 3.610, 1.938, 5.446]

    arm.move_to_joint_pose(start_jnt_pose)

    # pickup_jnt_pose = [3.501, 2.220, -0.101, 1.614, 3.887, 1.953, 5.883]
    pickup_jnt_pose = [3.846, 2.436, -0.412, 1.450, 3.926, 2.205, 5.446]

    for i in range(3):
      arm.move_to_joint_pose(pickup_jnt_pose)
      rospy.sleep(1)

    ## Grasp gripper
    gripper.close(force=150)
    rospy.sleep(1)

    ## Starting_point of trajectory
    # start_pose = Pose()
    # start_pose.position.x =  threshold_values(traj_to_follow[0][0]/scale, 0.8)
    # start_pose.position.y = threshold_values(traj_to_follow[0][1]/scale, 0.3)
    # start_pose.position.z = threshold_values(traj_to_follow[0][2]/scale + z_offset, 0.3)
    # start_pose.orientation.x = 0.671
    # start_pose.orientation.y = 0.644
    # start_pose.orientation.z = -0.233
    # start_pose.orientation.w = 0.284

    # arm.move_to_ee_pose(start_pose)
    for i in range(3):
      arm.move_to_joint_pose([-2.16739, 2.13966,-0.951256,2.62457,-2.78328,1.14412,-0.547369])

    rospy.sleep(2)

    poses.append(arm.get_FK(root="linear_actuator_link")[0].pose)

    ### Execute Trajectory
    for pt in traj_to_follow:
          pos_ = Pose()
          pos_.position.x = threshold_values(pt[0]/scale, 0.8)
          pos_.position.y = threshold_values(pt[1]/scale, 0.3)
          pos_.position.z = threshold_values(pt[2]/scale + z_offset, 0.3)
          pos_.orientation.x = 0.671
          pos_.orientation.y = 0.644
          pos_.orientation.z = -0.233
          pos_.orientation.w = 0.284
          poses.append(pos_)          

        # arm.plan_ee_pos(pos_)
        # rospy.sleep(2)

    (plan3, fraction) = arm.group[0].compute_cartesian_path(
                             poses,   # waypoints to follow
                             0.012,        # eef_step
                             0.0)         # jump_threshold

    print "fraction of sccuess: ", fraction

    ## Move to start_point of trajectory
    traj_start = list(plan3.joint_trajectory.points[0].positions)
    arm.move_to_joint_pose(traj_start)

    # Display planned traj
    display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = \
        arm.robot.get_current_state()
    display_trajectory.trajectory.append(plan3)
    display_trajectory_publisher.publish(display_trajectory)

    rospy.sleep(1)

    raw_input('Press Enter to Execute Trajectory')
    arm.move_robot(plan3)


    rospy.sleep(1)
    raw_input('Press Enter to open Gripper')
    gripper.open()

    # Start Position
    # arm.move_to_joint_pose(jnt_traj[0])
    # arm.move_to_joint_pose(top_traj[0])
    # arm.move_to_joint_pose(stapler_traj_2[0])

    # raw_input('Press Enter to start execution')

    # # Jnt Position
    # for jnt_pose in stapler_traj_2:
    #     arm.move_to_joint_pose(jnt_pose)
    #     rospy.sleep(2)

    # print "FINISHED!!!"

    # arm.move_to_joint_pose(side_traj[0])
    # rospy.sleep(2)
    # raw_input('Press Enter to start execution')

    # # Side Table
    # for jnt_pose in side_traj:
    #     arm.move_to_joint_pose(jnt_pose)
    #     rospy.sleep(2)

    # # End positin
    # # arm.move_to_joint_pose(top_traj[0])
    # arm.move_to_joint_pose(side_traj[0])
    # rospy.sleep(2)
