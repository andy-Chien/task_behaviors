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

class ImgMaskingClientState(EventState):
    '''
    Get the grasp pose from GQCNN server.

    -- mask_service     string                The Name of mask image service.
    -- namespace        string                The namespace choose which marker id to use.

    ># mask_img_msg      sensor_msgs/Image[]   The image msg of masked image,
                                                [0],[2] are color and depth mask for id1,
                                                [1],[3] are color and depth mask for id2.


    <= done 						          Get mask image.
    <= failed 						          Failed to get mask image.
    <= retry 						          Retry state.
    '''

    def __init__(self, namespace='', marker_id=1, 
                 create_depth_mask=False, update_mask=False, start_update_timer=False, 
                 stop_update_timer=False, mark_release=False, get_masked_img=False,
                 resolution_wide=516, resolution_high=386):
        '''
        Constructor
        '''
        super(ImgMaskingClientState, self).__init__(outcomes=['done', 'failed', 'retry'],
                                                output_keys=['mask_img_msg', 'img_info', 'marker_poses'])
        
        self._node = EventState._node
        ProxyServiceCaller._initialize(self._node)
        self._logger = self._node.get_logger().info

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._mask_service = '/' + namespace + '/image_masking'
        else:
            self._mask_service = '/image_masking' #image_masking

        self.namespace = namespace
        rq = GetMaskImage.Request
        self.mask_req = rq()
        self.mask_req.mask_id = marker_id
        self.mask_req.mode = 0
        if create_depth_mask:
            self.mask_req.mode |= rq.CREATE_DEPTH_MASK
        if update_mask:
            self.mask_req.mode |= rq.UPDATE_MARK_MASK
        if start_update_timer:
            self.mask_req.mode |= rq.START_UPDATE_TIMER
        if stop_update_timer:
            self.mask_req.mode |= rq.STOP_UPDATE_TIMER
        if mark_release:
            self.mask_req.mode |= rq.MARK_RELEASE
        if get_masked_img:
            self.mask_req.mode |= rq.GET_MASKED_IMG
            self.mask_req.resolution = [resolution_wide, resolution_high]

        self._masking_client = ProxyServiceCaller({self._mask_service: GetMaskImage})
        self.bridge = CvBridge()

        self._fail_count = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        
        if not self._masking_client.done(self._mask_service):
            self._logger('waiting mask image')
            return

        res = self._masking_client.result(self._mask_service)
        if not res.success:
            self._fail_count += 1
            if self._fail_count > 10000:
                self._logger("bad position, cannot get all ar_marker")
                return 'failed'
            else:
                return 'retry'					
        else:
            self._fail_count = 0
            rq = GetMaskImage.Request

            if self.mask_req.mode & rq.GET_MASKED_IMG:
                userdata.mask_img_msg = [res.rgb_img, res.depth_img, res.segmask]
                userdata.img_info = [res.rgb_camera_info, res.depth_camera_info]
                self._logger("Get Masked Image!")
                for p in res.marker_poses:
                    pos = p.position
                    quat = p.orientation
                    self._logger("pos = {}, {}, {}, quat = {}, {}, {}, {}".format(
                            pos.x, pos.y, pos.z, quat.w, quat.x, quat.y, quat.z
                        )
                    )

            if self.mask_req.mode & (rq.GET_MASKED_IMG | rq.UPDATE_MARK_MASK):
                userdata.marker_poses = res.marker_poses

            return 'done'


    def on_enter(self, userdata):
        self._masking_client.call_async(self._mask_service, self.mask_req)    

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass