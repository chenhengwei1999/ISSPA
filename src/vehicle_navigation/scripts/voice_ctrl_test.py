#!/usr/bin/env python3
# coding: utf-8

import Speech_Lib
import time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped

spe = Speech_Lib.Speech()
pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

def voice_pub_goal():
    rospy.init_node('voice_pub_goal_publisher', anonymous=True) # ROS节点初始化
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()
    # The location of the target point

    # The posture of the target point. z=sin(angle/2) w=cos(angle/2)
    
    
    
    while not rospy.is_shutdown():  
        speech_r = spe.speech_read()
        if speech_r == 19 : 
            print("goal to A")
            spe.void_write(speech_r)
            pose.pose.position.x = 2.15381097794
            pose.pose.position.y =  -5.02386903763
            pose.pose.orientation.z = 0.726492681307
            pose.pose.orientation.w = 0.687174202082
            pub_goal.publish(pose)
        elif speech_r == 20 :
            print("goal to B")
            spe.void_write(speech_r)
            pose.pose.position.x = 1.57744419575
            pose.pose.position.y = 4.8174996376
            pose.pose.orientation.z = -0.683335654604
            pose.pose.orientation.w = 0.730104364558
            pub_goal.publish(pose)
            
        elif speech_r == 21 :
            print("goal to C")
            spe.void_write(speech_r)
            pose.pose.position.x =  -1.08106160164
            pose.pose.position.y = 1.30198049545
            pose.pose.orientation.z =  -0.0132771070267
            pose.pose.orientation.w =  0.99991185533
            pub_goal.publish(pose)
        rate.sleep()
if __name__ == '__main__':
    try:
        voice_pub_goal()
    except rospy.ROSInterruptException:
        pass
    
            
            
