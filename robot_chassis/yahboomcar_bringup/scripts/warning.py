#!/usr/bin/env python
# encoding: utf-8
import rospy

if __name__ == '__main__':
    rospy.init_node("warning_node")
    r=rospy.Rate(0.2)
    while not rospy.is_shutdown():
        #print("车型错误！请检查.bashrc文件中的车型设置！！！")
        rospy.logwarn("车型错误！请检查.bashrc文件中的车型设置！！！")
        r.sleep()