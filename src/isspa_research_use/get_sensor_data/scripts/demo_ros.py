#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from autoware_msgs.msg import DetectedObjectArray


import numpy as np

from pcdet.models import build_network, load_data_to_gpu
from pcdet.config import cfg_from_yaml_file, cfg
from pcdet.datasets import DatasetTemplate
from pcdet.utils import common_utils

import glob

import torch


class LidarDetector(object):
    def __init__(self, sub_topic,sub_type,
                 pub_topic, pub_type,
                 cfg_file="",
                 ckpt_file=""):
        self.point_cloud_sub = rospy.Subscriber(sub_topic, sub_type, self.point_cloud_callback)
        self.detection_pub = rospy.Publisher(pub_topic, pub_type, queue_size=1)

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

                print(pred_dicts)

    def point_cloud_callback(self, raw_point_cloud):
        self.points_xyzi = self.pointcloud2_to_xyzi(raw_point_cloud)
        rospy.loginfo("Getting point cloud")


    def pointcloud2_to_xyzi(self, raw_point_cloud):
        points_list = []

        for data in read_points(raw_point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            # print("x: %f, y: %f, z: %f, intensity: %f" % (data[0], data[1], data[2], data[3]))
            # points_list.append([data[0], data[1], data[2], data[3], data[4]])
            points_list.append([data[0], data[1], data[2], data[3]])

        points_list = np.asarray(points_list, dtype=np.float32).reshape(-1, 4)
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

        print(point_cloud.shape)
        print(point_cloud[0, :])

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