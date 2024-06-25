#! /usr/env/bin python

import rospy
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import cv2

from datetime import datetime




def point_cloud_callback(msg):
    ...


class GetSensorStream(object):
    def __init__(self, node_name="get_sensor_stream", lidar_topic=None, camera_0_topic=None):
        # Initialize a Python ROS node
        rospy.init_node(node_name)

        # Create a subscriber to get specific kind of sensor data, e.g. lidar
        point_cloud_subscriber = rospy.Subscriber(lidar_topic, PointCloud2, self.point_cloud_callback)
        image_subscriber = rospy.Subscriber(camera_0_topic, Image, self.image_callback)


    def point_cloud_callback(self, msg):
        pass

    def image_callback(self):
        pass


    def listen(self):
        self.point_cloud = np.zeros()
        self.camera_0 = None



if __name__ == "__main__":
    sensor_stream = GetSensorStream(node_name="get_sensor_stream",
                                    lidar_topic="/lslidar_point_cloud",
                                    camera_0_topic="/camera_0/imgae_raw")

    while rospy.is_shutdown():
        sensor_stream.listen()
    
    rospy.spin()



    