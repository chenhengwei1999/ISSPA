#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

from sensor_msgs.point_cloud2 import read_points, create_cloud

import open3d as o3d

import sys
import os

import numpy as np
import cv2
from cv_bridge import CvBridge

from datetime import datetime



def pointcloud2_to_xyzi(msg):
    points_list = []

    for data in read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        # print("x: %f, y: %f, z: %f, intensity: %f" % (data[0], data[1], data[2], data[3]))
        # points_list.append([data[0], data[1], data[2], data[3], data[4]])
        points_list.append([data[0], data[1], data[2], data[3]])

    
    return np.array(points_list)


def print_pointcloud_fields(msg):
    print("Fields: ", [field.name for field in msg.fields])




def point_cloud_callback(msg):

    # Print fields of point cloud message
    # print_pointcloud_fields(msg)
    

    # Define timestamp of point cloud message to be stored
    header = msg.header
    timestamp_sec = header.stamp.secs

    # Define file path to store data
    current_file_path = os.path.abspath(sys.argv[0])
    parent_path = os.path.dirname(current_file_path)
    target_path = os.path.dirname(parent_path)
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_store_path = os.path.join(target_path, "scripts", "data", "point_cloud_outdoor", current_time) + ".ply"
    file_store_path_bin = os.path.join(target_path, "scripts", "data", "point_cloud_outdoor", current_time) + ".bin"

    print("File store path: ", file_store_path)
    
    # Get the number of points in the point cloud
    num_points = len(msg.data)
    

    # Get the points in the point cloud
    points = pointcloud2_to_xyzi(msg)
    rospy.loginfo("Shape of point cloud data: ")
    rospy.loginfo(points.shape)

    xyzi = points[:, :4]
    i = xyzi[:, 3] / 255.0    # 强度值
    xyzi[:, 3] = i

    print("max intensity: ", np.max(xyzi[:, 3]))
    print("min intensity: ", np.min(xyzi[:, 3]))

    print("max x: ", np.max(xyzi[:, 0]))
    print("min x: ", np.min(xyzi[:, 0]))

    print("max y: ", np.max(xyzi[:, 1]))
    print("min y: ", np.min(xyzi[:, 1]))

    print("max z: ", np.max(xyzi[:, 2]))
    print("min z: ", np.min(xyzi[:, 2]))

    xyz = xyzi[:, :3]  # 三维坐标

    print(xyzi.dtype)

    xyzi = xyzi.astype(np.float32)

    xyzi.tofile(file_store_path_bin)

    # 转换为Open3D的点云格式
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    # # 将强度值转换为颜色（这里简单地将强度映射到RGB，实际应用中可能需要根据强度的具体范围调整）
    colors = np.zeros((xyz.shape[0], 3))
    colors[:, 0] = i  # 使用强度值填充红色通道作为示例，你可以根据需要映射到其他颜色通道
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # # 保存为PLY文件
    o3d.io.write_point_cloud(file_store_path, pcd)

    print("Point cloud data stored in: ", file_store_path)



def camera_callback(msg):
    bridge = CvBridge()
    
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Define file path to store data
        current_file_path = os.path.abspath(sys.argv[0])
        parent_path = os.path.dirname(current_file_path)
        target_path = os.path.dirname(parent_path)
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        img_file_path = os.path.join(target_path, "scripts", "data", "images_outdoor", current_time) + ".png"

        # Save image
        cv2.imwrite(img_file_path, cv_image)
        rospy.loginfo("Saved image: {}".format(img_file_path))

    except Exception as e:
        rospy.logerr("Failed to save image: {}".format(e))



if __name__ == "__main__":

    rospy.init_node("get_sensor_data")
    rospy.loginfo("Node get_sensor_data started")
    point_cloud_sub_stream = rospy.Subscriber("/lslidar_point_cloud", PointCloud2, point_cloud_callback)
    camera_stream = rospy.Subscriber("/camera_1/image_raw", Image, camera_callback)
    
    rospy.spin()




