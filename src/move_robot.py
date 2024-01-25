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

        self.path = rospy.wait_for_message('path_topic', PoseArray)
        self.set_orientations(self.path)
        self.distance_tolerance = 0.2
        # Index to keep track of the current goal
        self.current_goal_index = 0

        # Send the first goal
        self.send_next_goal()
    

    def feedback_callback(self, feedback):
        # Monitor feedback, and decide when to send a new goal

        # Example: Check if the robot is close to the current goal
        current_pose = feedback.base_position.pose.position
        goal_pose = self.path.poses[self.current_goal_index].position
        distance_to_goal = ((current_pose.x - goal_pose.x) ** 2 +
                            (current_pose.y - goal_pose.y) ** 2) ** 0.5

        if distance_to_goal < 2:
            self.send_next_goal()

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
    
    def send_next_goal(self):
        # Check if there are more goals in the array
        if self.current_goal_index < len(self.path.poses)-1:
            # Create a new MoveBaseGoal for the next waypoint
            new_goal = MoveBaseGoal()
            new_goal.target_pose.header.frame_id = "odom"
            new_goal.target_pose.pose = self.path.poses[self.current_goal_index]

            # Send the new goal to the navigation stack
            self.move_base_client.send_goal(new_goal, feedback_cb=self.feedback_callback)

            # Increment the index for the next iteration
            self.current_goal_index += 1
            if self.current_goal_index == len(self.path.poses)-1:
                rospy.loginfo("All goals reached.")
        
    def set_orientations(self, pose_array):
    # Extract individual poses from the received PoseArray
        poses = pose_array.poses
        for i in range(len(poses)-1):
            pos1 = [poses[i].position.x, poses[i].position.y, poses[i].position.z]
            pos2 = [poses[i + 1].position.x, poses[i + 1].position.y, poses[i + 1].position.z]
            
            orientation = self.calculate_orientation(pos1, pos2)
            
            # Add the calculated orientation to the current pose
            poses[i].orientation.w = orientation.w
            poses[i].orientation.z = orientation.z

if __name__ == '__main__':
    # Create an instance of the Navigator class
    navigator = Navigator()

    # Keep the node running
    rospy.spin()
