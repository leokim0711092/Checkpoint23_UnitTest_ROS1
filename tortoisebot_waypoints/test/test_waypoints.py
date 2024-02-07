#! /usr/bin/env python3

import actionlib
from geometry_msgs.msg import Point
from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import rospy
import rosunit
import unittest
import rostest
import math

#import sys
PKG = 'tortoisebot_waypoints'
NAME = 'position_yaw_ros_unittest'


class TestWaypoint(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_orientation = Quaternion()
        self.current_position = Point()

        self.initial_x = 0
        self.initial_y = 0
        self.initial_yaw = 0
        self.desired_yaw = 0
        self.final_x = 0
        self.final_y = 0
        self.final_yaw = 0
        self.result = False
        self.x_diff =0
        self.y_diff =0
        self.yaw_diff =0

        self.execute()

    def execute(self):

        client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        client.wait_for_server()
        goal = WaypointActionGoal()
        dest = Point()
        dest.x = 0.5
        dest.y = 0.5
        goal.position = dest
        self.initial_x = self.current_position.x
        self.initial_y = self.current_position.y
        self.initial_yaw = self.euler_to_quaternion(self.current_orientation)
        self.desired_yaw = math.atan2(dest.y - self.initial_y, dest.x - self.initial_x)

        client.send_goal(goal)
        client.wait_for_result()

        self.final_x = self.current_position.x
        self.final_y = self.current_position.y
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        self.result = client.get_result()

        self.x_diff = abs(self.final_x - dest.x)
        self.y_diff = abs(self.final_y - dest.y)
        self.xy_deff = math.sqrt(self.x_diff*self.x_diff + self.y_diff*self.y_diff)
        self.yaw_diff = self.final_yaw - self.desired_yaw

    def odom_callback(self, msg):

        self.current_orientation = msg.pose.pose.orientation
        self.current_position = msg.pose.pose.position

    def euler_to_quaternion(self, msg):

        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def test_xy(self):
        
        self.assertTrue((-0.02 <= self.xy_deff <= 0.02), "Failure, XY error is over 0.03")

    
    def test_yaw(self):
        print("final yaw:", self.final_yaw)
        print("Desired yaw:", self.desired_yaw)

        
        self.assertTrue(((0.5 <= self.yaw_diff <= 0.5)), "Failure, Yaw error is over 0.5")



if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoint)