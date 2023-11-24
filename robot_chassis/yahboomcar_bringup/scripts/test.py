#!/usr/bin/env python2
# coding:utf-8
from Turn_Right import *
class Turn():
    def __init__(self):
        self.sub_traffic_sign = rospy.Subscriber('/traffic_logo', Int32, self.excute, queue_size=1)
        self.trun_right = YahboomCarPatrol()
    def excute(self,msg):
        if msg == 2:
            self.trun_right.process()
        else:
            self.trun_right.go()
if __name__ == '__main__':
    rospy.init_node("turn_node", anonymous=False)
    #trun_right.process()
    rospy.spin()
    print(6666)
