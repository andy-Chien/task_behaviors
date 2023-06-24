#!/usr/bin/env python3
import os
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from ros2_aruco_interfaces.srv import GetMaskImage
'''
Created on 24.02.2022

@author: TaiTing Tsai
'''

class GetMaskImageState(EventState):
    '''
    Get the grasp pose from GQCNN server.

    -- grasp_service    string      The Name of grasp planning service.

    ># grasp_posision   float[]     The position of grasp pose.
    ># grasp_quaternion float[]     The quaternion of grasp pose.

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    '''

    def __init__(self, namespace, mask_service):
        '''
        Constructor
        '''
        super(GetMaskImageState, self).__init__(outcomes=['done', 'failed', 'retry'],
                                            output_keys=['mask_imgmsg'])
        
        self._node = GetMaskImageState._node
        ProxyServiceCaller._initialize(self._node)
        # ProxySubscriberCached._initialize(self._node)
        self._logger = self._node.get_logger().info
        
        self._mask_service = mask_service #image_masking

        
        self.mask_req = GetMaskImage.Request()
        self._gqcnn_client = ProxyServiceCaller({self._mask_service: GetMaskImage})
        self.info_received = False
        self.color_image_received = False
        self.depth_image_received = False
        self.bridge = CvBridge()

        self._result = None
        
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

        self.mask_req.get_mask = True
        self._gqcnn_client.call_async(self._mask_service, self.mask_req)    

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)