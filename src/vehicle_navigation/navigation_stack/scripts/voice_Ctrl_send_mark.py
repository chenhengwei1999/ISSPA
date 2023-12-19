#!/usr/bin/env python
# encoding: utf-8

import rospy
from std_msgs.msg import Bool
from move_base_msgs.msg import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
import Speech_Lib
spe = Speech_Lib.Speech()


class Multipoint_navigation:
    def __init__(self):
        # 发布目标点 || Publish target point
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        # 订阅到达目标点的状态 || Subscribe to the status of reaching the target point
        #self.sub_goal_result = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.goal_result_callback)
        # 订阅初始化位置话题 || Subscribe to the initial pose topic
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)
        self.speech_r = 999

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        if msg.data:
            self.cancel_marker()
            self.pub_cmdVel.publish(Twist())


    def cancel_marker(self):
        # Clear marker
        self.markerArray = MarkerArray()
        marker = Marker()
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        self.pub_mark.publish(self.markerArray)
        self.InitialParam()
        self.pub_cancelgoal.publish(GoalID())


    def PubTargetPoint(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        # The location of the target point
        pose.pose.position.x = x
        pose.pose.position.y = y
        # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        self.pub_goal.publish(pose)
        
    def voice_pub_goal(self):
        self.speech_r = spe.speech_read()
        
        if self.speech_r == 19 :
            
            print("goal to A")
            spe.void_write(self.speech_r)
            #self.PubTargetPoint(3.24762392044,-1.04746031761)
# A
        elif self.speech_r == 20 :
            print("goal to B")
            spe.void_write(self.speech_r)
            #self.PubTargetPoint(1,3)
# B
        elif self.speech_r == 21 :
            print("goal to C")
            spe.void_write(self.speech_r)
            #self.PubTargetPoint(2,3)


if __name__ == '__main__':
    rospy.init_node('MultiPoint_navigation')
    goal = Multipoint_navigation()
    while True:
        goal.voice_pub_goal()
    
            
    rospy.spin()
