#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached, ProxyTransformListener
from gqcnn_interfaces.srv import GQCNNGraspPlanner
from sensor_msgs.msg import CameraInfo, Image
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
import numpy as np
import quaternion as qtn
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

'''
Created on 21.06.2023

@author: Andy Chien
'''

class GetBoxPointCloudState(EventState):
    '''
    Generate pointcloud for packing planning from depth img

    -- grasp_service    string      The Name of grasp planning service.

    ># grasp_posision   float[]     The position of grasp pose.
    ># grasp_quaternion float[]     The quaternion of grasp pose.

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(GetBoxPointCloudState, self).__init__(outcomes=['done'],
                                                    input_keys=['masked_depth_img', 'camera_info', 'marker_poses'],
                                                    output_keys=['box_pointcloud', 'camera_trans_box', 'box_size', 'frame'])
        
        self._node = GetBoxPointCloudState._node
        ProxySubscriberCached._initialize(self._node)
        ProxyTransformListener._initialize(self._node)
        self._logger = self._node.get_logger().info
        
        self.info_received = False
        self.color_image_received = False
        self.depth_image_received = False
        self.bridge = CvBridge()

        self.depthpcd = []
        self.cnt_depth_img = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        return 'done'


    def on_enter(self, userdata):
        depth_img = self.bridge.imgmsg_to_cv2(userdata.masked_depth_img, desired_encoding='32FC1')

        cam_intrinsic = o3d.camera.PinholeCameraIntrinsic(int(userdata.camera_info.width), 
                                                            int(userdata.camera_info.height),
                                                            float(userdata.camera_info.k[0]),
                                                            float(userdata.camera_info.k[4]),
                                                            float(userdata.camera_info.k[2]),
                                                            float(userdata.camera_info.k[5]))
        cam_intrinsic.intrinsic_matrix = [[float(userdata.camera_info.k[0]), 0, float(userdata.camera_info.k[2])], 
                                          [0, float(userdata.camera_info.k[4]), float(userdata.camera_info.k[5])], 
                                          [0, 0, 1]]
        
        mps = [np.array([x.position.x, x.position.y, x.position.z]) for x in userdata.marker_poses]
        center = np.mean(mps, axis=0)
        vec_x = mps[-1] - mps[0]
        vec_y = mps[1] - mps[0]
        vec_x_len = np.linalg.norm(vec_x)
        vec_y_len = np.linalg.norm(vec_y)
        if abs(np.dot(vec_x / vec_x_len, [1, 0, 0])) > abs(np.dot(vec_y / vec_y_len, [1, 0, 0])):
            vec_x, vec_y, vec_x_len, vec_y_len = vec_y, vec_x, vec_y_len, vec_x_len
        
        conter_pixel = [int(depth_img.shape[0] / 2), int(depth_img.shape[1] / 2)]
        center_depth = depth_img[conter_pixel[0]][conter_pixel[1]] - center[2]
        userdata.box_size = [vec_x_len, vec_y_len, center_depth]

        camera_2_box = np.mat(np.identity(4))
        camera_2_box[:3, 3] = center

        userdata.camera_trans_box = camera_2_box
        userdata.frame = userdata.masked_depth_img.header.frame_id
        depthpcd = o3d.open3d.geometry.PointCloud.create_from_depth_image(
            o3d.geometry.Image(depth_img.astype(np.float32) * 0.001), 
            cam_intrinsic, depth_trunc=0)

        userdata.box_pointcloud = depthpcd.transform(camera_2_box.T)

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)