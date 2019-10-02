#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Point32
from sensor_msgs.msg import PointCloud
import tf

prefix = "/head_camera_remote/"
topic_name = prefix + "rgb/object_pose"
topic_type = PoseStamped

rospy.init_node('frame_converter')
pub = rospy.Publisher(prefix + 'pose_fridge_handle', PoseStamped, queue_size=1)
listener = tf.TransformListener()

def callback(msg):
    pose_target = listener.transformPose("/base_link", msg) 
    pub.publish(pose_target)

rospy.Subscriber(topic_name, topic_type, callback)
rospy.spin()


