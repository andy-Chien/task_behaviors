#!/usr/bin/env python3
import os
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from gqcnn_interfaces.srv import GQCNNGraspPlannerSegmask
from sensor_msgs.msg import CameraInfo, Image
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
import numpy as np
from matplotlib import pyplot as plt

# from autolab_core import (Point, Logger, BinaryImage, CameraIntrinsics,
#                           ColorImage, DepthImage)
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

    def __init__(self, pj_grasp_service='/gqcnn_pj/grasp_planner_segmask', 
                 suc_grasp_service='/gqcnn_suc/grasp_planner_segmask'):
        '''
        Constructor
        '''
        super(GQCNNGraspPlanState, self).__init__(outcomes=['done', 'failed', 'retry', 'nothing'],
                                                  input_keys=['mask_img_msg', 'camera_info_msg'],
                                                  output_keys=['pj_pose', 'suc_pose', 'frame', 'pj_qv', 'suc_qv'])
        
        self._node = GQCNNGraspPlanState._node
        ProxyServiceCaller._initialize(self._node)
        self._logger = self._node.get_logger().info
        
        self._pj_grasp_service = pj_grasp_service
        self._suc_grasp_service = suc_grasp_service
        
        self.gqcnn_req = GQCNNGraspPlannerSegmask.Request()
        self._gqcnn_pj_client = ProxyServiceCaller({self._pj_grasp_service: GQCNNGraspPlannerSegmask})
        self._gqcnn_suc_client = ProxyServiceCaller({self._suc_grasp_service: GQCNNGraspPlannerSegmask})
        self.bridge = CvBridge()

        self.has_pj_server = False
        self.has_suc_server = False

        self._fail_count = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if not self.has_pj_server and not self.has_suc_server:
            return 'failed'

        if (self.has_suc_server and not self._gqcnn_suc_client.done(self._suc_grasp_service))\
            or (self.has_pj_server and not self._gqcnn_pj_client.done(self._pj_grasp_service)):
            self._logger('waiting GQCNN')
            return
        result = []
        if self.has_pj_server:
            result.append(self._gqcnn_pj_client.result(self._pj_grasp_service))
        if self.has_suc_server:
            result.append(self._gqcnn_suc_client.result(self._suc_grasp_service))

        for r in result:
            img = self.bridge.imgmsg_to_cv2(r.grasp.thumbnail, desired_encoding='32FC1')
            self._logger('center_px = {}, q_value = {}, z_value = {}'.format(
                r.grasp.center_px, r.grasp.q_value, img[int(r.grasp.center_px[1]), int(r.grasp.center_px[0])]))
            # cv2.circle(img, np.array(r.grasp.center_px, dtype=np.uint32), 5, (255), -1)
            # plt.imshow(img, cmap='gray', vmin=0.0, vmax=2.0)
            # plt.show()

        qv = [r.grasp.q_value for r in result]
        if np.all(np.array(qv) < 0.01):
            self._fail_count += 1
            if self._fail_count > 10:
                self._logger('[GQCNN Grasp Plan State]: Grasp plan failed')
                return 'failed'
            else:
                return 'retry'					
        else:
            for r in result:
                p, q = r.grasp.pose.position, r.grasp.pose.orientation
                self._logger('grasp_pos = {}, {}, {}, qtn = {}, {}, {}, {}'.format(
                    p.x, p.y, p.z, q.w, q.x, q.y, q.z))
                
            userdata.frame = self.image_frame
            
            if self.has_pj_server and self.has_suc_server:
                userdata.pj_pose = result[0].grasp.pose
                userdata.suc_pose = result[1].grasp.pose
                userdata.pj_qv = result[0].grasp.q_value
                userdata.suc_qv = result[1].grasp.q_value
            elif self.has_pj_server:
                userdata.suc_pose = None
                userdata.pj_pose = result[0].grasp.pose
                userdata.pj_qv = result[0].grasp.q_value
            elif self.has_suc_server:
                userdata.pj_pose = None
                userdata.suc_pose = result[0].grasp.pose
                userdata.suc_qv = result[0].grasp.q_value
            else:
                userdata.pj_pose = None
                userdata.suc_pose = None
            
            return 'done'


    def on_enter(self, userdata):
        self.gqcnn_req.color_image = userdata.mask_img_msg[0]
        self.gqcnn_req.depth_image = userdata.mask_img_msg[1]
        self.gqcnn_req.segmask = userdata.mask_img_msg[2]
        self.gqcnn_req.camera_info = userdata.camera_info_msg[1]
        self.image_frame = self.gqcnn_req.depth_image.header.frame_id

        self.has_pj_server = self._gqcnn_pj_client.is_available(self._pj_grasp_service)
        self.has_suc_server = self._gqcnn_suc_client.is_available(self._suc_grasp_service)

        if self.has_pj_server:
            self._gqcnn_pj_client.call_async(self._pj_grasp_service, self.gqcnn_req)
        if self.has_suc_server:
            self._gqcnn_suc_client.call_async(self._suc_grasp_service, self.gqcnn_req)

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)