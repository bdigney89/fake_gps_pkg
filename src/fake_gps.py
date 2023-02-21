#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header
import numpy as np


class fake_gps_node:

    def __init__(self):
        pub_topic_name ="/fake_gps"
        self.pub = rospy.Publisher(pub_topic_name, NavSatFix, queue_size=2)
        timer_period = 0.5  # seconds

    def publish_fake_gps(self):
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "fake_gps1"

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            # Position in degrees.
            msg.latitude = 57.047218
            msg.longitude = 9.920100

            # Altitude in metres.
            msg.altitude = 1.15

            msg.position_covariance[0] = 0
            msg.position_covariance[4] = 0
            msg.position_covariance[8] = 0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.pub.publish(msg)
            self.best_pos_a = None


if __name__ == '__main__':
    node_name ="fake_gps"
    rospy.init_node(node_name)
    fgps= fake_gps_node()
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        fgps.publish_fake_gps()
        rate.sleep()
    

