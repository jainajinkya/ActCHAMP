#!/usr/bin/env python
import tf2_ros
import geometry_msgs.msg
import math
import tf
import tf_conversions
import rospy
import numpy as np
from experiments.srv import frame_data, frame_dataResponse

parent_frames = []
object_frames = []
poses = []
brs = []
processing = False


class broacast_frame():
    def __init__(self, parent, child):
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        self.t.header.frame_id = parent
        self.t.child_frame_id = child
        self.rate = rospy.Rate(10.0)

    def broadcast(self):
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)
        self.rate.sleep()

    def check_quaternion(self, q_array):
        q = np.array([q_array.x, q_array.y, q_array.z, q_array.w])
        q /= np.linalg.norm(q)
        quat = geometry_msgs.msg.Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        return quat


    def setPose(self, msg):
        self.t.transform.translation.x = msg.position.x
        self.t.transform.translation.y = msg.position.y
        self.t.transform.translation.z = msg.position.z
        self.t.transform.rotation = self.check_quaternion(msg.orientation)
        return


def handle_frame_server(req):
    global parent_frames, object_frames, poses, brs, processing
    processing = True

    if req.reset:
        parent_frames = req.parent_frames
        object_frames = req.child_frames
        poses = req.poses
    else:
        parent_frames.extend(req.parent_frames)
        object_frames.extend(req.child_frames)
        poses.extend(req.poses)

    brs = []

    for i in range(len(object_frames)):
        brs.append(broacast_frame(parent_frames[i], object_frames[i]))
        brs[-1].setPose(poses[i].pose)

    processing = False

    res = frame_dataResponse()
    res.success = True
    return res


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster', anonymous=True)
    serv = rospy.Service(
        'broadcast_frame', frame_data, handle_frame_server)

    while(not rospy.is_shutdown()):
        if not processing:
            for br in brs:
                br.broadcast()
