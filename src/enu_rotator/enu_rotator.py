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
        self.publisher = rospy.Publisher('/gps', Odometry)
        self.subscriber = rospy.Subscriber('/enu', Odometry, self.enu_callback)
        self.subscriber = rospy.Subscriber('/imu_data', Imu, self.imu_callback)
        self.rotated_enu = None
        self.rotation = None


        rospy.init_node('enu_rotator')

    def publish_loop(self):
        while not rospy.is_shutdown():
            if self.rotated_enu is not None:
                odom = Odometry()

                odom.header.frame_id = "odom"
                odom.header.stamp = time()
                odom.pose.pose.position.x = self.rotated_enu[0]
                odom.pose.pose.position.y = self.rotated_enu[1]
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = 0
                odom.pose.pose.orientation.y = 0
                odom.pose.pose.orientation.z = 0
                odom.pose.pose.orientation.w = 1

                odom.child_frame_id = "base_link"
                odom.twist.twist.linear.x = 0
                odom.twist.twist.linear.y = 0
                odom.twist.twist.linear.z = 0
                odom.twist.twist.angular.x = 0
                odom.twist.twist.angular.y = 0
                odom.twist.twist.angular.z = 0

                self.publisher.publish(odom)

    def enu_callback(self, msg):
        if self.rotation is not None:
            self.rotated_enu = numpy.matrix([msg.pose.pose.position.x, msg.pose.pose.position.y]) * self.rotation

    def imu_callback(self, msg):
        o = msg.orientation
        r, p, y = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        theta = y * 180 / math.pi
        self.rotation = numpy.matrix([math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)])


if __name__ == '__main__':
    er = EnuRotator()
    er.publish_loop()