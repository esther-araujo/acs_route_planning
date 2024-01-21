#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, PoseArray
import math

class Navigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('navigation_example', anonymous=True)

        # Create an action client for the move_base server
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)

        # Wait for the move_base action server to come up
        self.move_base_client.wait_for_server()

        # Subscribe to the path_topic
        self.path_subscriber = rospy.Subscriber('path_topic', PoseArray, self.path_callback)

    def calculate_orientation(self, current_x, current_y, next_x, next_y):
        # Calculate the angle between the two points
        angle = math.atan2(next_y - current_y, next_x - current_x)

        # Convert the angle to quaternion representation
        orientation_w = math.cos(angle / 2)
        orientation_z = math.sin(angle / 2)

        return orientation_w, 0, 0, orientation_z  # Assuming a 2D orientation, so roll and pitch are 0

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

    def path_callback(self, pose_array):
        # Extract individual poses from the received PoseArray
        poses = pose_array.poses
        for i in range(len(poses)-1):
            current_pose = poses[i]
            next_pose = poses[i+1]

            current_x, current_y = current_pose.position.x, current_pose.position.y
            next_x, next_y = next_pose.position.x, next_pose.position.y

            # Calculate orientation for the next goal
            orientation = self.calculate_orientation(current_x, current_y, next_x, next_y)

            # Create a PoseStamped object for the next goal
            next_goal_pose = PoseStamped()
            next_goal_pose.header.frame_id = 'odom'  # Adjust the frame_id based on your map frame
            next_goal_pose.header.stamp = rospy.Time.now()
            next_goal_pose.pose.position.x = next_x
            next_goal_pose.pose.position.y = next_y
            next_goal_pose.pose.orientation.w = orientation[0]
            next_goal_pose.pose.orientation.z = orientation[3]

            # Call the navigate_to_goal function for each pose
            self.navigate_to_goal(next_goal_pose)

if __name__ == '__main__':
    # Create an instance of the Navigator class
    navigator = Navigator()

    # Keep the node running
    rospy.spin()
