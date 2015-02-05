#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

pub_goal = None
pub_convert = None
sub_convert = None

def handle_xy_goal(req):
    p = PoseStamped()
    p.header = req.header
    p.pose = req.pose.pose
    rospy.loginfo("GPS goal converted to xy: " + str(p.pose.position.x) + ", " + str(p.pose.position.y) + ". Publishing..")
    global pub_goal
    pub_goal.publish(p)


def handle_coordinate_goal(req):
    rospy.loginfo("Received GPS goal: " + str(req.longitude) + ", " + str(req.latitude))
    pub_convert.publish(req)

def handle_fix(req):
    global pub_convert
    pub_convert.publish(req)

class CoordinateGoalTransformer:

    def __init__(self):
        rospy.init_node('coordinate_goal_transformer')

        #send first coordinates to converter
        global pub_convert
        pub_convert = rospy.Publisher("/navsat_transform_node_requests/gps_convert_to_xy", NavSatFix, queue_size=10)

        sub_convert = rospy.Subscriber("~/gps_sim", NavSatFix, handle_fix)
        rospy.sleep(1)
        sub_convert.unregister()
        rospy.sleep(0.5)
        rospy.loginfo("Initial coordinates sent, now waiting for goals")

        #stop listening to real gps and listen to goal topic
        rospy.Subscriber("/navsat_transform_node_requests/gps_converted_to_xy", Odometry, handle_xy_goal)

        global pub_goal
        pub_goal = rospy.Publisher("~goal_xy", PoseStamped, queue_size=10)
        rospy.Subscriber("~goal_coordinates", NavSatFix, handle_coordinate_goal)

        rospy.spin()

if __name__ == "__main__":
    try:
        CoordinateGoalTransformer()
    except rospy.ROSInterruptException:
        pass
