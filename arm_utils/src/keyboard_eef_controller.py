#!/usr/bin/env python

from __future__ import print_function
import copy
import numpy as np
import threading
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from vector_msgs.msg import GripperCmd

''' Utils '''
def TransformStamped_to_PoseStamped(trans):
    pose = PoseStamped()
    pose.header = trans.header
    pose.pose.position.x = trans.transform.translation.x
    pose.pose.position.y = trans.transform.translation.y
    pose.pose.position.z = trans.transform.translation.z
    pose.pose.orientation.x = trans.transform.rotation.x
    pose.pose.orientation.y = trans.transform.rotation.y
    pose.pose.orientation.z = trans.transform.rotation.z
    pose.pose.orientation.w = trans.transform.rotation.w
    return pose

class tf_publisher():
    def __init__(self, rate=10.):
        self.tf_broadcasters = []
        self.tf_rate = rospy.Rate(rate)
        self.tf_array = []
        self.tf_broadcasters = []
        self.quit = False
        self.tf_thread = threading.Thread(target=self.publish_tf)
        self.tf_thread.start()

    def update_tf(self, tf_array):
        self.tf_array = tf_array
        self.tf_broadcasters = [tf2_ros.TransformBroadcaster() for x in self.tf_array]

    def publish_tf(self):
        while not rospy.is_shutdown() and not self.quit:
            for i, x in enumerate(self.tf_array):
                x.header.stamp = rospy.Time.now()
                if len(self.tf_broadcasters) > 0:
                    self.tf_broadcasters[i].sendTransform(x)
            self.tf_rate.sleep()


''' MAIN keyboard controller class'''

class keyboard_controller():
    def __init__(self, threshold=0.02, reference_frame="linear_actuator_link", arm_name='right'):
        self.reference_frame = reference_frame
        self.arm_name = arm_name
        self.arm = ArmMoveIt(planning_frame=self.reference_frame, _arm_name=self.arm_name)
        self.gripper_publisher = rospy.Publisher("/vector/"+ self.arm_name +"_gripper/cmd", GripperCmd, queue_size=1)
        self.threshold = threshold
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.visualize = False
        self.send_transform = True
        self.tf_broadcaster = tf_publisher()
        print("============ Reference frame: {}".format(self.arm.group[0].get_planning_frame()))
        self.print_help()       
    
    def execute_motion(self, key):
        # cur_ee_pos = self.arm.get_FK(req_frame=self.arm_name+'_ee_link')[0]
        cur_ee_pos = TransformStamped_to_PoseStamped(self.tfBuffer.lookup_transform(self.reference_frame, self.arm_name+'_ee_link', rospy.Time(0)))
        print("Current ee pose: {}".format(cur_ee_pos))      
        new_pose = copy.copy(cur_ee_pos)
        
        if key == "w":
            new_pose.pose.position.x += self.threshold
        elif key == "s":
            new_pose.pose.position.x -= self.threshold
        elif key == "a":
            new_pose.pose.position.y += self.threshold
        elif key == "d":
            new_pose.pose.position.y -= self.threshold
        elif key == "e":
            new_pose.pose.position.z += self.threshold
        elif key == "x":
            new_pose.pose.position.z -= self.threshold
        elif key in ["o","open"]:
            self.gripper_controller(key)
        elif key in ["l", "close"]:
            self.gripper_controller(key)
        elif key in ["h", "help"]:
            self.print_help()

        print("New ee pose: {}".format(new_pose))
        if self.send_transform:
            self.broadcast_transform(new_pose)
        
        plan_ok = False
        while not plan_ok:
            cur_jnt_pose = np.array(self.arm.get_current_pose(group_id=0))
            plan = self.arm.plan_pose(new_pose, is_joint_pos=False)
            for pt in plan.joint_trajectory.points:
                new_jnt_pose = np.array(pt.positions)
                if np.amax(new_jnt_pose - cur_jnt_pose) > 0.015:
                    plan_ok = False
                    print("cur_jnt_pose: {}".format(cur_jnt_pose))
                    print("new_jnt_pose: {}".format(new_jnt_pose))
                    print("diff: {}".format(np.amax(new_jnt_pose - cur_jnt_pose)))
                    raw_input('Failed to plan. Press ENTER to replan')
                    break
                else:
                    plan_ok = True
                    cur_jnt_pose = copy.copy(new_jnt_pose)

        return self.arm.move_robot(plan)

    def gripper_controller(self, key, force=100.):
        msg = GripperCmd()
        if key in ["o","open"]:
            msg.position = 1.0
        elif key in ["l", "close"]:
            msg.position = 0.
            msg.force = force

        self.gripper_publisher.publish(msg)

    def broadcast_transform(self, new_pose):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.reference_frame
        t.child_frame_id = "target_frame"
        t.transform.translation.x = new_pose.pose.position.x
        t.transform.translation.y = new_pose.pose.position.y
        t.transform.translation.z = new_pose.pose.position.z
        t.transform.rotation.x = new_pose.pose.orientation.x
        t.transform.rotation.y = new_pose.pose.orientation.y
        t.transform.rotation.z = new_pose.pose.orientation.z
        t.transform.rotation.w = new_pose.pose.orientation.w
        self.tf_broadcaster.update_tf([t])

        
    def run(self):
        while not rospy.is_shutdown():
            key = raw_input('Enter Command: ')
            if key == 'q':
                self.tf_broadcaster.quit = True
                break
            else:
                self.execute_motion(key)
                
        print("Exiting keyboard control")
        
        
    def print_help(self):
        print("##################################################")
        print("Help function. \nNote: 1. All key presses for linear motion correspond to motion \
        of 2.5 cm in the specified direction. Angular motions correspond to 0.25 rad motion.\n \
        2. All motions are wrt base_frame specified in main function")
        print("Use following keys to generate end-effector motion")
        print("w : Forward")
        print("s : Backward")
        print("d : Right")
        print("a : Left")
        print("e : Up")
        print("x : Down")
        print("o,open : Open gripper")
        print("l,close : Close gripper")
        print("q : quit")
        print("##################################################")


if __name__ == "__main__":
    rospy.init_node('keyboard_eef_controller', anonymous=True)
    controller = keyboard_controller()
    controller.run()
        