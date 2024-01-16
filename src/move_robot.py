#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseArray

class MoveRobot(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()

    def moveToDest(self, pose):
        self.goal.target_pose.header.frame_id = "odom"
        self.goal.target_pose.pose = pose
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

def pose_array_callback(msg, moveRobot):
    for pose in msg.poses:
        moveRobot.moveToDest(pose.position)

if __name__ == "__main__":
    rospy.init_node("move_base_client")
    moveRobot = MoveRobot()

    # Subscribe to the topic that provides PoseArray
    rospy.Subscriber("path_topic", PoseArray, pose_array_callback, moveRobot)

    rospy.spin()
