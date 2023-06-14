#!/usr/bin/env python3
import os
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from gqcnn_interfaces.srv import GQCNNGraspPlanner
from sensor_msgs.msg import CameraInfo, Image
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from autolab_core import (Point, Logger, BinaryImage, CameraIntrinsics,
                          ColorImage, DepthImage)
'''
Created on 24.02.2022

@author: Andy Chien
'''

class GQCNNGraspPlanState(EventState):
    '''
    Get the grasp pose from GQCNN server.

    -- grasp_service    string      The Name of grasp planning service.

    ># grasp_posision   float[]     The position of grasp pose.
    ># grasp_quaternion float[]     The quaternion of grasp pose.

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    '''

    def __init__(self, grasp_service, depth_camera_info, color_image_topic, depth_image_topic):
        '''
        Constructor
        '''
        super(GQCNNGraspPlanState, self).__init__(outcomes=['done', 'failed', 'retry'],
                                            output_keys=['grasp_position', 'grasp_quaternion'])
        
        self._node = GQCNNGraspPlanState._node
        ProxyServiceCaller._initialize(self._node)
        ProxySubscriberCached._initialize(self._node)
        self._logger = self._node.get_logger().info
        
        self._grasp_service = grasp_service
        self._depth_camera_info = depth_camera_info
        self._color_image_topic = color_image_topic
        self._depth_image_topic = depth_image_topic
        
        self.gqcnn_req = GQCNNGraspPlanner.Request()
        self._gqcnn_client = ProxyServiceCaller({self._grasp_service: GQCNNGraspPlanner})
        self._depth_camera_info_subscriber_ = ProxySubscriberCached({self._depth_camera_info : CameraInfo})
        self._color_image_subscriber_ = ProxySubscriberCached({self._color_image_topic : Image})
        self._depth_image_subscriber_ = ProxySubscriberCached({self._depth_image_topic : Image})
        self.info_received = False
        self.color_image_received = False
        self.depth_image_received = False
        self.bridge = CvBridge()

        self.depth_camera_info = None
        # self.color_img = Image()
        # self.depth_img = Image()
        self.color_img = None
        self.depth_img = None

        self._result = None
        self._fail_count = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if not self.info_received or not self.color_image_received or not self.depth_image_received:
            return 'retry'
        
        if not self._gqcnn_client.done(self._grasp_service):
            # self._gqcnn_client.call_async(self._grasp_service, self.gqcnn_req) 
            self._logger('waiting GQCNN')
            return 

        self._result = self._gqcnn_client.result(self._grasp_service)
        if self._result.grasp.q_value < 0.1:
            self._fail_count += 1
            if self._fail_count > 10:
                self._logger('[GQCNN Grasp Plan State]: Grasp plan failed')
                return 'failed'
            else:
                return 'retry'					
        else:
            p, q = self._result.grasp.pose.position, self._result.grasp.pose.orientation
            userdata.grasp_position = [p.x, p.y, p.z]
            userdata.grasp_quaternion = [q.w, q.x, q.y, q.z]
            self._logger('grasp_position = {}'.format(userdata.grasp_position))
            self._logger('grasp_quaternion = {}'.format(userdata.grasp_quaternion))
            return 'done'


    def on_enter(self, userdata):
        self.info_received = False
        self.color_image_received = False
        self.depth_image_received = False
        if self._depth_camera_info_subscriber_.has_msg(self._depth_camera_info):
            self.get_depth_camera_Info(self._depth_camera_info_subscriber_.get_last_msg(self._depth_camera_info))
        if self._color_image_subscriber_.has_msg(self._color_image_topic):
            self.get_color_image_Info(self._color_image_subscriber_.get_last_msg(self._color_image_topic))
        if self._depth_image_subscriber_.has_msg(self._depth_image_topic):
            self.get_depth_image_Info(self._depth_image_subscriber_.get_last_msg(self._depth_image_topic))



        if self.info_received and self.color_image_received and self.depth_image_received:
            np.save(os.getcwd()+"/depth_npy.npy",self.depth_img)


            depth_npy = os.getcwd()+"/depth_npy.npy"
            depth_img = DepthImage.open(depth_npy, frame=self.depth_camera_info.header.frame_id)
            camera_intr = CameraIntrinsics(self.depth_camera_info.header.frame_id, self.depth_camera_info.k[0],
                                            self.depth_camera_info.k[4], self.depth_camera_info.k[2],
                                            self.depth_camera_info.k[5], 0.0, self.depth_camera_info.height,
                                            self.depth_camera_info.width,)
            # self.gqcnn_req = GQCNNGraspPlanner.Request()
            self.gqcnn_req.color_image = self.color_img
            self.gqcnn_req.depth_image = depth_img.rosmsg
            self.gqcnn_req.camera_info = camera_intr.rosmsg

            self._gqcnn_client.call_async(self._grasp_service, self.gqcnn_req)    

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)

    def get_depth_camera_Info(self, data):
        self.depth_camera_info = data
        self.info_received = True

    def get_color_image_Info(self, data):
        color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        color_image = cv2.resize(color_image, (516, 386))
        color_imagemm = self.bridge.cv2_to_imgmsg(color_image,"bgr8")
        self.color_img = color_imagemm
        self.color_image_received = True

    def get_depth_image_Info(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        depth_image = cv2.resize(depth_image*0.001, (516, 386))
        self.depth_img = depth_image
        self.depth_image_received = True
