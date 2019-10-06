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

# closure
def gen_callback(topic_name, topic_type):
    pub = rospy.Publisher(topic_name, topic_type, queue_size = 1)
    global msg 
    msg = None
    def inner(msg_):
        global msg
        msg = msg_
        pub.publish(msg)
    return inner

cb_fridge = gen_callback('fridge_pose', PoseStamped)
cb_handle = gen_callback('handle_pose', PoseStamped)

rospy.Subscriber(topic_fridge_name, topic_type, cb_fridge)
rospy.Subscriber(topic_handle_name, topic_type, cb_handle)
rospy.spin()


