#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Import the message type you expect to receive
from geometry_msgs.msg import PoseStamped

# Define a global variable to track whether a message has been received
message_received = False

# Callback function to handle incoming messages on the topic
def message_callback(msg):
    global message_received
    # Set the global flag to indicate that a message has been received
    message_received = True

def move_robot():
    rospy.init_node('move_robot_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a publisher for the move_base_simple/goal topic
    goal_pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
    
    # Create a PoseStamped message for the goal
    goal = PoseStamped()
    goal.header.frame_id = "map"  # Set the frame_id to the map frame
    goal.pose.position.x = 1.0    # Set the X position of the goal
    goal.pose.position.y = 2.0    # Set the Y position of the goal
    goal.pose.orientation.w = 1.0 # Set the orientation (quaternion) of the goal

    # Create a Twist message to specify the desired linear and angular velocity
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.2  # Adjust the linear velocity (m/s)
    cmd_vel_msg.angular.z = 0.0  # Adjust the angular velocity (rad/s)

    # Subscribe to the topic where you expect to receive a message
    rospy.Subscriber('/move_robot', String, message_callback)

    # Wait for a message to be received before moving the robot
    while not message_received:
        rate.sleep()
    
    goal_pub.publish(goal)

    # Start moving the robot after receiving a message
    # while not rospy.is_shutdown():
    #     
    #     cmd_vel_pub.publish(cmd_vel_msg)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
