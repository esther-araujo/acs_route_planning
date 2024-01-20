#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class GoalSender:
    def __init__(self):
        rospy.init_node('goal_sender', anonymous=True)

        rospy.Subscriber('/path_topic', PoseArray, self.goal_callback)

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

    def goal_callback(self, pose_array):
        rospy.loginfo("Received PoseArray with {} poses".format(len(pose_array.poses)))

        for goal_pose in pose_array.poses:
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = "map"
            move_base_goal.target_pose.pose = goal_pose

            rospy.loginfo("Sending goal: {}".format(goal_pose))
            self.move_base_client.send_goal(move_base_goal)

            self.move_base_client.wait_for_result()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        goal_sender = GoalSender()
        goal_sender.run()
    except rospy.ROSInterruptException:
        pass
