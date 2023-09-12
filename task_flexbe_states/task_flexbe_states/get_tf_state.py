#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from flexbe_core.proxy.proxy_transform_listener import ProxyTransformListener
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
'''
Created on 21.06.2023

@author: TaiTing Tsai
'''

class GetTfState(EventState):
    '''
    get static tf.

    -- parent_frame    string[]      The Name of tf parent frame.
    -- child_frame     string[]      The Name of tf child frame.

    <= done 						Robot move done.
    '''

    def __init__(self, parent_frame, child_frame):
        '''
        Constructor
        '''
        super(GetTfState, self).__init__(outcomes=['done', 'failed'],
                                        output_keys=['transform_list'])
        
        self._node = GetTfState._node
        self._logger = self._node.get_logger().info

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self._node)
        ProxyTransformListener._initialize(self._node)
        self.tf_listener = ProxyTransformListener().listener()
        self.tf_buffer = self.tf_listener.buffer
        self.transform_list = []
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        for i in range(len(self.parent_frame)):     
            try:
                trans_tf = self.tf_buffer.lookup_transform(
                    self.parent_frame[i], 
                    self.child_frame[i], 
                    rclpy.time.Time())
                self.transform_list.append(trans_tf)
                self._logger('-------------------------------{}'.format(self.transform_list))
            except TransformException as ex:
                self.transform_list = []
                Logger.logwarn('lookupTransform {}, {} failed!'.format(self.parent_frame[i], self.child_frame[i]))
                return "failed"

        userdata.transform_list = self.transform_list
        self.cnt = 0
        return 'done'


    def on_enter(self, userdata):
        self.transform_list = []
        pass

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)
