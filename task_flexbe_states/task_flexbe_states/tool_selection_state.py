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
from copy import deepcopy
import quaternion as qtn

# from autolab_core import (Point, Logger, BinaryImage, CameraIntrinsics,
#                           ColorImage, DepthImage)
'''
Created on 24.02.2022

@author: Andy Chien
'''

DIS = 0.05

class ToolSelectionState(EventState):
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
        super(ToolSelectionState, self).__init__(outcomes=['done', 'failed'],
                                                  input_keys=['marker_poses', 'pj_pose', 'suc_pose', 
                                                              'frame', 'pj_qv', 'suc_qv', 'curr_tool', 'img_info', 'img'],
                                                  output_keys=['target_pose', 'tar_tool'])
        
        self._node = ToolSelectionState._node
        
        self._logger = self._node.get_logger().info
        self.cv_gridge = CvBridge()
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        sp, pp = deepcopy(userdata.suc_pose), deepcopy(userdata.pj_pose)
        sq, pq = userdata.suc_qv, userdata.pj_qv
        if not sp is None and not pp is None:
            mp = [np.array([x.position.x, x.position.y, x.position.z]) for x in userdata.marker_poses]
            min_z = np.min(mp, axis=0)[2] - 0.5

            if sp.position.z < min_z - DIS:
                sp = None
            if pp.position.z < min_z - DIS:
                pp = None
            
            vec = [mp[i-1] - mp[i] for i in len(mp)]
            vec = [v / np.linalg.norm(v) for v in vec]
            pland_d = [-1 * sum(v * p) for v, p in zip(vec, mp)]

            sp_min_d = DIS
            pp_min_d = DIS
            sp_min_d_vec = None
            pp_min_d_vec = None

            for v, d in zip(vec, pland_d):
                sd = sum(np.array([sp.position.x, sp.position.y, sp.position.z]) * v) + d
                pd = sum(np.array([pp.position.x, pp.position.y, pp.position.z]) * v) + d
                if sd < sp_min_d:
                    sp_min_d = sd
                    sp_min_d_vec = v
                if pd < pp_min_d:
                    pp_min_d = pd
                    pp_min_d_vec = v

            if sp_min_d < -1 * DIS:
                sp = None
            elif sp_min_d < 0:
                sp.position.x -= sp_min_d * sp_min_d_vec[0]
                sp.position.y -= sp_min_d * sp_min_d_vec[1]
                sp.position.z -= sp_min_d * sp_min_d_vec[2]
            if pp_min_d < -1 * DIS:
                pp = None
            if pp_min_d < 0:
                pp.position.x -= pp_min_d * pp_min_d_vec[0]
                pp.position.y -= pp_min_d * pp_min_d_vec[1]
                pp.position.z -= pp_min_d * pp_min_d_vec[2]

            s_factor = (sp_min_d + DIS) / (2 * DIS) if sp_min_d < DIS else 1
            p_factor = (pp_min_d + DIS) / (2 * DIS) if pp_min_d < DIS else 1

            sq *= s_factor
            pq *= p_factor


        if not sp is None and not pp is None:
            if 'suc' in userdata.curr_tool:
                if sq > 0.5:
                    userdata.target_pose = sp
                    userdata.tar_tool = 'suction'
                elif pq > sq * 2:
                    userdata.target_pose = pp
                    userdata.tar_tool = 'pj'
                else:
                    userdata.target_pose = sp
                    userdata.tar_tool = 'suction'

            elif 'pj' in userdata.curr_tool:
                if pq > 0.5:
                    userdata.target_pose = pp
                    userdata.tar_tool = 'pj'
                elif sq > pq * 2:
                    userdata.target_pose = sp
                    userdata.tar_tool = 'suction'
                else:
                    userdata.target_pose = pp
                    userdata.tar_tool = 'pj'
            
        else:
            if not sp is None:
                userdata.target_pose = sp
            elif not pp is None:
                userdata.target_pose = pp
            else:
                return 'failed'
            
        
        if userdata.tar_tool == 'pj':
            p = np.array([pp.position.x, pp.position.y, pp.position.z])
            q = qtn.quaternion(pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z)
            r = qtn.as_rotation_matrix(q)
            t = np.identity(4)
            t[:3, :3] = r
            t[:3, 3] = p
            p0 = []
            p0.append(np.matmul(t, (np.array([ 0.01,  0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0]))))
            p0.append(np.matmul(t, (np.array([ 0.01, -0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0]))))
            p0.append(np.matmul(t, (np.array([-0.01, -0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0]))))
            p0.append(np.matmul(t, (np.array([-0.01,  0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0]))))

            p1 = []
            p1.append(np.matmul(t, (np.array([ 0.01,  0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0]))))
            p1.append(np.matmul(t, (np.array([ 0.01, -0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0]))))
            p1.append(np.matmul(t, (np.array([-0.01, -0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0]))))
            p1.append(np.matmul(t, (np.array([-0.01,  0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0]))))

            depth_img = self.cv_gridge.imgmsg_to_cv2(userdata.img[1], desired_encoding='16UC1')
            camera_intrinsics = np.reshape(userdata.img_info[1].k, (3,3))
            avg_d0 = self.calculate_average_depth(depth_img, p0, camera_intrinsics) / 1000.0
            avg_d1 = self.calculate_average_depth(depth_img, p1, camera_intrinsics) / 1000.0

            max_depth = min(avg_d0, avg_d1)
            if pp.position.z > max_depth:
                pp.position.z = max_depth

        return 'done'
    

    def calculate_average_depth(self, d_img, corners, camera_intrinsics):
        # Extract the camera intrinsic parameters
        fx = camera_intrinsics[0, 0]
        fy = camera_intrinsics[1, 1]
        cx = camera_intrinsics[0, 2]
        cy = camera_intrinsics[1, 2]
        
        # Convert the corners from world frame to pixel coordinates
        pixel_corners = []
        for corner in corners:
            x, y, z = corner
            u = int(fx * x / z + cx)
            v = int(fy * y / z + cy)
            pixel_corners.append((u, v))
        
        # Extract the pixel coordinates of the corners
        x1, y1 = pixel_corners[0]
        x2, y2 = pixel_corners[1]
        x3, y3 = pixel_corners[2]
        x4, y4 = pixel_corners[3]
        
        # Calculate the minimum and maximum x and y values
        min_x = min(x1, x2, x3, x4)
        max_x = max(x1, x2, x3, x4)
        min_y = min(y1, y2, y3, y4)
        max_y = max(y1, y2, y3, y4)
        
        # Clip the coordinates within the image boundaries
        min_x = max(0, min_x)
        max_x = min(d_img.shape[1] - 1, max_x)
        min_y = max(0, min_y)
        max_y = min(d_img.shape[0] - 1, max_y)
        
        # Extract the region of interest (ROI) from the depth image
        roi = d_img[min_y:max_y+1, min_x:max_x+1]
        
        # Calculate the average depth within the ROI
        average_depth = np.mean(roi)
        
        return average_depth