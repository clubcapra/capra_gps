#!/usr/bin/env python
import csv
from capra_gps.srv import RotateRequest, Rotate

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import math

import rospy
import tf


class WaypointOrientation:
    def __init__(self):
        rospy.init_node('waypoint_orientation')

        self.waypoints = list()
        self.sensitivity = rospy.get_param('~sensitivity', 1.5)
        self.next_waypoint = None
        self.position = None
        self.next_waypoint_publisher = rospy.Publisher("/next_waypoint", Pose)
        self.robot_pose_subscriber = rospy.Subscriber('/robot_pose_ekf/odom', PoseWithCovarianceStamped, self._kalman_listener)

        f = open(rospy.get_param('~file'), 'rt')
        try:
            reader = csv.reader(f)

            for waypoint in reader:
                for coord in waypoint:
                    self.waypoints.append(coord.split(';'))
        finally:
            f.close()

    def publish_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(self.waypoints) != 0:
                if self.position is not None:
                    if self.next_waypoint is None or (self.position.pose.pose.position.x - self.next_waypoint.position.x) ** 2 + (self.position.pose.pose.position.y - self.next_waypoint.position.y) ** 2 < self.sensitivity ** 2:
                        waypoint = self.waypoints.pop(0)

                        rotate = rospy.ServiceProxy('/gps/rotate', Rotate)
                        request = RotateRequest()
                        request.point.pose.pose.position.x = waypoint[0]
                        request.point.pose.pose.position.y = waypoint[1]
                        response = rotate(request)

                        self.next_waypoint = Pose()
                        self.next_waypoint.position.x = response.rotated.pose.pose.position.x
                        self.next_waypoint.position.y = response.rotated.pose.pose.position.y

                    quaternion = tf.transformations.quaternion_from_euler(0, 0, math.atan((self.position.pose.pose.position.x - self.next_waypoint.position.x) / (self.position.pose.pose.position.y - self.next_waypoint.position.y)))

                    self.next_waypoint.orientation.x = quaternion[0]
                    self.next_waypoint.orientation.y = quaternion[1]
                    self.next_waypoint.orientation.z = quaternion[2]
                    self.next_waypoint.orientation.w = quaternion[3]

                    self.next_waypoint_publisher.publish(self.next_waypoint)
            r.sleep()

    def _kalman_listener(self, msg):
        self.position = msg


if __name__ == '__main__':
    er = WaypointOrientation()
    er.publish_loop()