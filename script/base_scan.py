#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

topic_name = "/base_scan"
topic_type = LaserScan

rospy.init_node('base_scan_analyzer')
pub = rospy.Publisher('basescan_range_min', Float32, queue_size=1)

def callback(msg):
    print "hoge"
    ranges_ = np.array(msg.ranges)
    ranges = ranges_[np.where(~np.isnan(msg.ranges))]
    range_min = ranges.min()
    data = Float32(data = range_min)
    pub.publish(range_min)

rospy.Subscriber(topic_name, topic_type, callback)
rospy.spin()

