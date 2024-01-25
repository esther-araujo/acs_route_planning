#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, PoseArray, Quaternion
from math import atan2
from tf.transformations import quaternion_from_euler

class Navigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('move_robot', anonymous=True)

        # Create an action client for the move_base server
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Wait for the move_base action server to come up
        self.move_base_client.wait_for_server()

        path = rospy.wait_for_message('path_topic', PoseArray)

        self.exec_path(path)

    def calculate_orientation(self, pos1, pos2):
        # Calculate the direction vector between two positions
        direction_vector = [pos2[0] - pos1[0], pos2[1] - pos1[1], pos2[2] - pos1[2]]

        # Normalize the direction vector
        norm = sum([x**2 for x in direction_vector])**0.5
        direction_vector = [x/norm for x in direction_vector]

        # Compute the quaternion based on the direction vector
        roll, pitch, yaw = 0.0, 0.0, atan2(direction_vector[1], direction_vector[0])
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        return Quaternion(*quaternion)

    def navigate_to_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        # Send the goal and wait for the result
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        # Check if the goal was successful
        if self.move_base_client.get_state() == 3:  # State 3 corresponds to SUCCEEDED
            rospy.loginfo("Navigation to the goal point successful!")
        else:
            rospy.logwarn("Failed to reach the goal point.")

    def exec_path(self, pose_array):
        # Extract individual poses from the received PoseArray
        poses = pose_array.poses
        for i in range(len(poses)-1):
            current_pose = poses[i]
            pos1 = [poses[i].position.x, poses[i].position.y, poses[i].position.z]
            pos2 = [poses[i + 1].position.x, poses[i + 1].position.y, poses[i + 1].position.z]
            orientation = self.calculate_orientation(pos1, pos2)
            # Calculate orientation for the next goal

            # Create a PoseStamped object for the next goal
            next_goal_pose = PoseStamped()
            next_goal_pose.header.frame_id = 'odom'  # Adjust the frame_id based on your map frame
            next_goal_pose.header.stamp = rospy.Time.now()
            next_goal_pose.pose = current_pose
            next_goal_pose.pose.orientation.w = orientation.w
            next_goal_pose.pose.orientation.z = orientation.z

            # Call the navigate_to_goal function for each pose
            self.navigate_to_goal(next_goal_pose)

if __name__ == '__main__':
    # Create an instance of the Navigator class
    navigator = Navigator()

    # Keep the node running
    rospy.spin()
