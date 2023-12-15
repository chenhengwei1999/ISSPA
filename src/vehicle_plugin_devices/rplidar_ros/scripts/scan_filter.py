#!/usr/bin/env python
# coding:utf-8
import math
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import numpy as np
RAD2DEG = 180 / math.pi


class scan_compression:
    def __init__(self):
        self.bridge = CvBridge()
        self.laserProj = LaserProjection()
        self.shielding_angle = rospy.get_param('~shielding_angle', 90)
        self.pub = rospy.Publisher("scan", LaserScan, queue_size=1000)
        self.laserSub = rospy.Subscriber("scan_raw", LaserScan, self.laserCallback, queue_size=1000)

    def laserCallback(self, data):
        if not isinstance(data, LaserScan): return
        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = data.header.frame_id
        laser_scan.angle_increment = data.angle_increment
        laser_scan.time_increment = data.time_increment
        laser_scan.intensities = data.intensities
        laser_scan.scan_time = data.scan_time
        laser_scan.angle_min = data.angle_min
        laser_scan.angle_max = data.angle_max
        laser_scan.range_min = data.range_min
        laser_scan.range_max = data.range_max
        for i in range(len(np.array(data.ranges))):
            angle = (data.angle_min + data.angle_increment * i) * RAD2DEG
            if abs(angle) >= self.shielding_angle: laser_scan.ranges.append(data.ranges[i])
            else: laser_scan.ranges.append(float("inf"))
        self.pub.publish(laser_scan)

if __name__ == '__main__':
    rospy.init_node('scan_filter', anonymous=False)
    pt2img = scan_compression()
    rospy.spin()
