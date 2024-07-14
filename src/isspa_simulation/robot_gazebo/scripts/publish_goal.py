#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def create_goal_message(x, y, z):
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"  # or the appropriate frame
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = z
    goal_msg.pose.orientation.w = 1.0  # assuming no rotation, adjust if necessary
    return goal_msg

def publish_goal(x, y, z):
    rospy.init_node('goal_publisher')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    goal_msg = create_goal_message(x, y, z)
    rospy.sleep(1)  # give some time for publisher to set up
    pub.publish(goal_msg)
    rospy.loginfo("Published goal: ({}, {}, {})".format(x, y, z))
    # rospy.spin()


if __name__ == "__main__":
    x, y, z = 1.0, 2.0, 0.0
    publish_goal(x, y, z)
    