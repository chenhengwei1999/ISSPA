#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from visualization_msgs.msg import Marker, MarkerArray


import numpy as np

from pcdet.models import build_network, load_data_to_gpu
from pcdet.config import cfg_from_yaml_file, cfg
from pcdet.datasets import DatasetTemplate
from pcdet.utils import common_utils

import glob

import torch


import open3d as o3d



class LidarDetector(object):
    def __init__(self, sub_topic,sub_type,
                 pub_topic, pub_type,
                 cfg_file="",
                 ckpt_file=""):
        self.point_cloud_sub = rospy.Subscriber(sub_topic, sub_type, self.point_cloud_callback)
        self.detection_pub = rospy.Publisher(pub_topic, pub_type, queue_size=1)
        self.marker_pub = rospy.Publisher("detection_markers", MarkerArray, queue_size=1)

        self.logger = common_utils.create_logger()

        # Read the cfg and other parameters
        cfg_from_yaml_file(cfg_file, cfg)

        # Create the dataset
        self.demo_ros_dataset = DemoDataset(dataset_cfg=cfg.DATA_CONFIG,
                                            class_names=cfg.CLASS_NAMES,
                                            training=False,
                                            logger=self.logger)
        print("Load the demo_ros_dataset")

        # Initilize the model
        self.model = build_network(model_cfg=cfg.MODEL,
                                   num_class=len(cfg.CLASS_NAMES),
                                   dataset=self.demo_ros_dataset)
        
        self.model.load_params_from_file(filename=ckpt_file,
                                         to_cpu=True,
                                         logger=self.logger)
        self.model.cuda()
        self.model.eval()

        # Initialize Open3D visualizer
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        # self.pcd = o3d.geometry.PointCloud()
        # self.vis.add_geometry(self.pcd)
        # self.bbox_lines = []
        # self.view_control = self.vis.get_view_control()


    def update_point_cloud(self, ):
        pass

    def load_model(self, ):
        pass

    def model_forward(self, ):
        # Load model structrue
        if self.points_xyzi is not None:
            with torch.no_grad():
                data_dict = self.demo_ros_dataset.get_data(self.points_xyzi)
                data_dict = self.demo_ros_dataset.collate_batch([data_dict])
                load_data_to_gpu(data_dict)
                pred_dicts, _ = self.model.forward(data_dict)

                # print(pred_dicts)
                # print(data_dict['points'].shape)

                # Extract point cloud and predictions
                points = data_dict['points'][:, 1:].cpu().numpy()
                pred_boxes = pred_dicts[0]['pred_boxes'].cpu().numpy()

                # Update Open3D visualizer
                # self.update_visualization(points, pred_boxes)

                # Publish detection results
                self.publish_detections(pred_boxes)

    def update_visualization(self, points, pred_boxes):

        # Save the current view parameters
        view_params = self.view_control.convert_to_pinhole_camera_parameters()

        self.pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        self.pcd.colors = o3d.utility.Vector3dVector(np.tile(points[:, 3:4], (1, 3)))

        # Remove old bounding boxes
        for bbox in self.bbox_lines:
            self.vis.remove_geometry(bbox)
        self.bbox_lines = []

        # Add new bounding boxes
        for box in pred_boxes:
            bbox = o3d.geometry.OrientedBoundingBox()
            bbox.center = box[:3]
            bbox.extent = box[3:6]
            bbox.color = (1, 0, 0)
            self.bbox_lines.append(bbox)
            self.vis.add_geometry(bbox)

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

        # Restore the view parameters
        self.view_control.convert_from_pinhole_camera_parameters(view_params)


    def publish_detections(self, pred_boxes):
        detection_array = DetectedObjectArray()
        detection_array.header.stamp = rospy.Time.now()
        detection_array.header.frame_id = "laser"

        marker_array = MarkerArray()

        for i, box in enumerate(pred_boxes):
            detected_object = DetectedObject()
            detected_object.pose.position.x = box[0]
            detected_object.pose.position.y = box[1]
            detected_object.pose.position.z = box[2]
            detected_object.dimensions.x = box[3]
            detected_object.dimensions.y = box[4]
            detected_object.dimensions.z = box[5]
            detected_object.label = "detected_object"
            detected_object.score = 1.0  # Assuming a confidence score of 1.0 for simplicity

            detection_array.objects.append(detected_object)

            # Create a marker for visualization
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detection"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = box[0]
            marker.pose.position.y = box[1]
            marker.pose.position.z = box[2]
            marker.scale.x = box[3]
            marker.scale.y = box[4]
            marker.scale.z = box[5]
            marker.color.a = 0.5  # Transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.detection_pub.publish(detection_array)
        self.marker_pub.publish(marker_array)



    def point_cloud_callback(self, raw_point_cloud):
        self.points_xyzi = self.pointcloud2_to_xyzi(raw_point_cloud)
        rospy.loginfo_once("Getting point cloud")


    def pointcloud2_to_xyzi(self, raw_point_cloud):
        points_list = []

        for data in read_points(raw_point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            # print("x: %f, y: %f, z: %f, intensity: %f" % (data[0], data[1], data[2], data[3]))
            # points_list.append([data[0], data[1], data[2], data[3], data[4]])
            points_list.append([data[0], data[1], data[2], data[3]])

        points_list = np.asarray(points_list, dtype=np.float32).reshape(-1, 4)
        points_list[:, 3] = points_list[:, 3] / 255

        return points_list


class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin',):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger)
        

    def __len__(self):
        return None

    def __getitem__(self, index=None):

        # input_dict = {
        #     'points': self.points_xyzi,
        #     'frame_id': index,
        # }

        # data_dict = self.prepare_data(data_dict=input_dict)
        return None
    
    def get_data(self, point_cloud):

        # print(point_cloud.shape)
        # print(point_cloud[0, :])

        input_dict = {
            'points': point_cloud,
            'frame_id': 1,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict
        


def lidar_detection():
    rospy.init_node("lidar_detection")
    lidar_detector = LidarDetector(sub_topic="point_cloud_raw",
                                   sub_type=PointCloud2,
                                   pub_topic="lidar_detection_results",
                                   pub_type=DetectedObjectArray,
                                   cfg_file="cfgs/kitti_models/pointrcnn.yaml",
                                   ckpt_file="/home/chw/Public/model_zoo/pcdet/pointrcnn_7870.pth")

    while not rospy.is_shutdown():
        lidar_detector.model_forward()


if __name__ == "__main__":
    lidar_detection()