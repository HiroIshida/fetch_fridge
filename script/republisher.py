#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
import tf

prefix = "/head_camera_remote/"
topic_fridge_name = prefix + "rgb/fridge_pose"
topic_handle_name = prefix + "rgb/handle_pose"
topic_type = PoseStamped

rospy.init_node('republisher')

class Republisher:
    def __init__(self, topic_name, topic_name_new, topic_type):
        self.msg = None
        self.sub = rospy.Subscriber(topic_name, topic_type, self._callback)
        self.pub = rospy.Publisher(topic_name_new, topic_type, queue_size = 1)

    def _callback(self, msg):
        self.msg = msg

    def publish(self):
        if self.msg is not None:
            self.pub.publish(self.msg)

rep1 = Republisher(topic_fridge_name, 'fridge_pose', PoseStamped)
rep2 = Republisher(topic_handle_name, 'handle_pose', PoseStamped)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    rep1.publish()
    rep2.publish()
    r.sleep()


