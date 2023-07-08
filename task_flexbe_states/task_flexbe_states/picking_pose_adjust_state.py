#!/usr/bin/env python3
import os
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from gqcnn_interfaces.srv import GQCNNGraspPlannerSegmask
from sensor_msgs.msg import CameraInfo, Image
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from matplotlib import pyplot as plt
from copy import deepcopy
import quaternion as qtn
from math import acos

# from autolab_core import (Point, Logger, BinaryImage, CameraIntrinsics,
#                           ColorImage, DepthImage)
'''
Created on 24.02.2022

@author: Andy Chien
'''

DIS = 0.1

class PickingPoseAdjustState(EventState):
    '''
    Get the grasp pose from GQCNN server.

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
        super(PickingPoseAdjustState, self).__init__(outcomes=['done', 'failed'],
                                                  input_keys=['marker_poses', 'target_pose', 'retry_cnt'],
                                                  output_keys=['target_pose', 'retry_cnt'])
        
        self._node = PickingPoseAdjustState._node
        
        self._logger = self._node.get_logger().info
        self.cv_gridge = CvBridge()
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        userdata.retry_cnt += 1
        if userdata.retry_cnt > 10:
            return 'failed'

        sp = deepcopy(userdata.target_pose)
        mp = [np.array([x.position.x, x.position.y, x.position.z]) for x in userdata.marker_poses]

        close_vec = []
        if sp != None:
            vec = [mp[i-1] - mp[i] for i in range(len(mp))]
            vec = [v / np.linalg.norm(v) for v in vec]
            pland_d = [-1 * np.dot(v, p) for v, p in zip(vec, mp)]

            
            for v, d in zip(vec, pland_d):
                sd = np.dot([sp.position.x, sp.position.y, sp.position.z], v) + d
                if sd < DIS:
                    close_vec.append(deepcopy(v))

        quat = qtn.quaternion(sp.orientation.w, sp.orientation.x, sp.orientation.y, sp.orientation.z)
        z_vec = qtn.as_rotation_matrix(quat).transpose()[2]
        for v in close_vec:
            rv = np.cross(z_vec, v) * -0.1
            quat *= qtn.from_rotation_vector(rv)
        if v == []:
            rv = np.cross(z_vec, [0,0,1]) * 0.1
            quat *= qtn.from_rotation_vector(rv)

        target_pose = PoseStamped()
        target_pose.pose.position.x = sp.position.x
        target_pose.pose.position.y = sp.position.y
        target_pose.pose.position.z = sp.position.z
        target_pose.pose.orientation.w = quat.w
        target_pose.pose.orientation.x = quat.x
        target_pose.pose.orientation.y = quat.y
        target_pose.pose.orientation.z = quat.z
        userdata.target_pose = target_pose
        return 'done'