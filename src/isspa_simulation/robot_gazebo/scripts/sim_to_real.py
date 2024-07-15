#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
import time

def drive_forward(distance, speed):
    # Calculate the time to drive based on distance and speed
    duration = distance / speed

    # Initialize the node
    rospy.init_node('drive_forward', anonymous=True)
    rospy.sleep(2)

    # Publisher to send velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message for forward motion
    move_cmd = Twist()
    move_cmd.linear.x = speed  # Set linear speed (m/s)
    move_cmd.angular.z = 0    # No angular velocity

    # Time control
    start_time = rospy.Time.now().to_sec()

    rospy.loginfo("Moving forward")
    # Loop to publish the velocity command
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(move_cmd)
        rospy.sleep(0.1)  # Sleep to avoid flooding the topic

    # Stop the robot after the loop
    move_cmd.linear.x = 0
    pub.publish(move_cmd)

def rotate_90_degrees(radius, linear_speed):
    # Angular speed (v = r * omega)
    angular_speed = linear_speed / radius

    # Time to rotate 90 degrees (pi/2 radians)
    duration = (math.pi / 2) / angular_speed

    # Initialize the node
    rospy.init_node('rotate_90_degrees', anonymous=True)

    # Publisher to send velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message for rotation
    move_cmd = Twist()
    move_cmd.linear.x = linear_speed   # Set linear speed (m/s)
    move_cmd.angular.z = angular_speed # Set angular velocity

    # Time control
    start_time = rospy.Time.now().to_sec()

    # Loop to publish the velocity command
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(move_cmd)
        rospy.sleep(0.1)  # Sleep to avoid flooding the topic

    # Stop the robot after the loop
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    pub.publish(move_cmd)



if __name__ == '__main__':
    try:
        # Adjust the distance (meters) and speed (m/s) as needed
        # drive_forward(distance=1.0, speed=0.2)
        rotate_90_degrees(radius=1.0, linear_speed=0.5)
    except rospy.ROSInterruptException:
        pass