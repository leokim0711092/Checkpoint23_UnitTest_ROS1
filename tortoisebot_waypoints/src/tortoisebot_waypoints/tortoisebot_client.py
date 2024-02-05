#! /usr/bin/env python3 
import rospy

import actionlib
from geometry_msgs.msg import Point

from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal

def Waypoint_client():
    client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
    _sub_odom = rospy.Subscriber('/odom', Odometry, _clbk_odom)


    client.wait_for_server()
 

    goal = WaypointActionGoal()
    pose = Point()
    pose.x = 0.5
    pose.y = 0.5

    goal.position = pose

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
         rospy.init_node('tortoisebot_as_client_py')
         result = Waypoint_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")