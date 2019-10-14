#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Pose
from geometry_msgs.msg import PoseStamped, Point32
from fetch_fridge.msg import PoseStampedBooled
from sensor_msgs.msg import PointCloud
import tf
from tbtop_square.msg import Projected

prefix = "/head_camera_remote/"
topic_fridge_name = prefix + "rgb/fridge_pose"
topic_handle_name = prefix + "rgb/handle_pose"
topic_type = PoseStamped

rospy.init_node('republisher')

def compute_time(msg_time):
    sec = msg_time.secs + msg_time.nsecs * 10 ** (-9)
    return sec

def isValidTimeSequence(time_list):
    for elem in time_list:
        if elem is None:
            return False
    second = rospy.get_time()
    if (abs(time_list[-1] - second) > 1.5):
        return False
    if (abs(time_list[-1] - time_list[0]) > 2.0):
        return False
    return True

class PoseQueue:
    def __init__(self, N):
        self.N = N
        self.data_position = [np.zeros(3) for n in range(N)]
        self.data_orientation = [np.zeros(4) for n in range(N)]
        self.data_time = [None for n in range(N)]

    def push(self, msg):
        tmp1 = self.data_position[1:self.N]
        tmp2 = self.data_orientation[1:self.N]
        tmp3 = self.data_time[1:self.N]
        msg_pose = msg.pose 
        posision_new = np.array([
            msg_pose.position.x,
            msg_pose.position.y,
            msg_pose.position.z])
        orientation_new = np.array([
            msg_pose.orientation.x,
            msg_pose.orientation.y,
            msg_pose.orientation.z,
            msg_pose.orientation.w])
        time_new = compute_time(msg.header.stamp)
        tmp1.append(posision_new)
        tmp2.append(orientation_new)
        tmp3.append(time_new)
        self.data_position = tmp1
        self.data_orientation = tmp2
        self.data_time = tmp3

    def mean(self):
        position_mean = [np.mean(np.array([s[i] for s in self.data_position])) for i in range(3)]
        orientation_mean_ = [np.mean(np.array([s[i] for s in self.data_orientation])) for i in range(4)]
        orientation_mean = orientation_mean_/np.linalg.norm(orientation_mean_)
        point = Point(x = position_mean[0], y = position_mean[1], z = position_mean[2])
        orientation = Quaternion(
                x = orientation_mean[0], 
                y = orientation_mean[1], 
                z = orientation_mean[2], 
                w = orientation_mean[3]) 
        pose = Pose(position = point, orientation = orientation)
        boolean = Bool(data = isValidTimeSequence(self.data_time))
        return pose, boolean

class Republisher:
    def __init__(self, topic_name, topic_name_new):
        self.sub = rospy.Subscriber(topic_name, PoseStamped, self._callback)
        self.pub = rospy.Publisher(topic_name_new, PoseStampedBooled, queue_size = 1)
        self.queue = PoseQueue(5)
        self.header = None

    def _callback(self, msg):
        self.queue.push(msg)
        self.header = msg.header

    def publish(self):
        if self.header is not None: 
            pose, boolean = self.queue.mean()
            msg_ps = PoseStamped(pose = pose, header = self.header)
            msg = PoseStampedBooled(ps = msg_ps, isvalid = boolean)
            self.pub.publish(msg)

rep1 = Republisher(topic_fridge_name, 'fridge_pose')
rep2 = Republisher(topic_handle_name, 'handle_pose')

r = rospy.Rate(10)
while not rospy.is_shutdown():
    rep1.publish()
    rep2.publish()
    r.sleep()


