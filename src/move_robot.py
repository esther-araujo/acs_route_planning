#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def navigate_to_pose(pose_stamped):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose = pose_stamped

    client.send_goal(goal)
    client.wait_for_result()

def pose_array_callback(data):
    for pose_stamped in data.poses:
        navigate_to_pose(pose_stamped)

if __name__ == '__main__':
    rospy.init_node('navigation_node')
    rospy.Subscriber('path_topic', PoseArray, pose_array_callback)
    rospy.spin()
