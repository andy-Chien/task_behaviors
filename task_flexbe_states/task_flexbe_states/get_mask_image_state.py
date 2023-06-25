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

    -- mask_service     string                The Name of mask image service.
    -- namespace        string                The namespace choose which marker id to use.

    ># mask_imgmsg      sensor_msgs/Image[]   The image msg of masked image,
                                                [0],[2] are color and depth mask for id1,
                                                [1],[3] are color and depth mask for id2.


    <= done 						          Get mask image.
    <= failed 						          Failed to get mask image.
    <= retry 						          Retry state.
    '''

    def __init__(self, mask_service, namespace=''):
        '''
        Constructor
        '''
        super(GetMaskImageState, self).__init__(outcomes=['done', 'failed', 'retry'],
                                                output_keys=['mask_imgmsg'])
        
        self._node = GetMaskImageState._node
        ProxyServiceCaller._initialize(self._node)
        self._logger = self._node.get_logger().info

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._mask_service = '/' + namespace + mask_service
        else:
            self._mask_service = mask_service #image_masking

        self.namespace = namespace
        self.mask_req = GetMaskImage.Request()
        self._gqcnn_client = ProxyServiceCaller({self._mask_service: GetMaskImage})
        self.bridge = CvBridge()

        self._result = None
        self._fail_count =0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        
        if not self._gqcnn_client.done(self._mask_service):
            self._logger('waiting mask image')
            return 

        self._result = self._gqcnn_client.result(self._mask_service)
        if "bad" in self._result.image_condition:
            self._fail_count += 1
            if self._fail_count > 10:
                self._logger("bad position, cannot get all ar_marker")
                return 'failed'
            else:
                return 'retry'					
        else:
            self._fail_count =0
            # userdata.mask_imgmsg = [self._result.mask_image[0],self._result.mask_image[1]]
            if '1' in self.namespace:
                userdata.mask_imgmsg = [self._result.mask_image[0],self._result.mask_image[2]]
            elif '2' in self.namespace:
                userdata.mask_imgmsg = [self._result.mask_image[1],self._result.mask_image[3]]
            self._logger("Get Masked Image!")
            return 'done'


    def on_enter(self, userdata):

        self.mask_req.get_mask = True
        self._gqcnn_client.call_async(self._mask_service, self.mask_req)    

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass