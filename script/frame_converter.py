#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
import tf

prefix = "/head_camera_remote/"
topic_fridge_name = prefix + "rgb/fridge_pose"
topic_handle_name = prefix + "rgb/handle_pose"
topic_type = PoseStamped

rospy.init_node('frame_converter')
pub_fridge = rospy.Publisher(prefix + 'fridge_pose_base', PoseStamped, queue_size=1)
pub_handle = rospy.Publisher(prefix + 'handle_pose_base', PoseStamped, queue_size=1)
listener = tf.TransformListener()

def callback_fridge(msg):
    pose_target = listener.transformPose("/base_link", msg) 
    pub_fridge.publish(pose_target)

def callback_handle(msg):
    pose_target = listener.transformPose("/base_link", msg) 
    pub_handle.publish(pose_target)

rospy.Subscriber(topic_fridge_name, topic_type, callback_fridge)
rospy.Subscriber(topic_handle_name, topic_type, callback_handle)
rospy.spin()


