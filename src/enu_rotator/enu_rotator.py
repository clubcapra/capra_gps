#!/usr/bin/env python

import rospy
import math
import numpy
import time
import tf

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class EnuRotator:
    def __init__(self):
        self.publisher = rospy.Publisher('/vo', Odometry)
        self.subscriber = rospy.Subscriber('/gps/enu', Odometry, self.enu_callback)
        self.subscriber = rospy.Subscriber('/imu_datum', Imu, self.imu_callback)
        self.rotated_enu = None
        self.rotation = None
        self.has_new_data = False
        self.x = 0
        self.y = 0
        self.theta = 0
        self.has_theta = False
        rospy.init_node('enu_rotator')

    def publish_loop(self):
        while not rospy.is_shutdown():
            if self.has_theta and self.has_new_data:
                odom = Odometry()

                odom.header.frame_id = "odom"
                odom.header.stamp = rospy.get_rostime()
                odom.pose.pose.position.y = self.y#-self.rotated_enu.tolist()[0][0]
                odom.pose.pose.position.x = self.x#-self.rotated_enu.tolist()[0][1]
                odom.pose.pose.orientation.w = 1
                odom.child_frame_id = "base_link"
                self.has_new_data = False
                self.publisher.publish(odom)
            else:
                time.sleep(1.0 / 1000)

    def enu_callback(self, msg):
        if self.has_theta:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            d = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
            self.y = math.cos(self.theta) * d
            self.x = -math.sin(self.theta) * d
            self.has_new_data = True

    def imu_callback(self, msg):
        o = msg.orientation
        r, p, y = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.theta = -(y + math.pi)
        self.has_theta = True
#       

if __name__ == '__main__':
    er = EnuRotator()
    er.publish_loop()