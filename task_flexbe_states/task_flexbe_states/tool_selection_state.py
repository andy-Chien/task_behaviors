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

DIS = 0.06
PJ_MIN_DIS = 0.03
SUC_MIN_DIS = 0.01

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
                                                              'frame', 'pj_qv', 'suc_qv', 'curr_tool', 'img_info', 'img', 'fail_cnt'],
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
        mp = [np.array([x.position.x, x.position.y, x.position.z]) for x in userdata.marker_poses]
        min_z = np.min(mp, axis=0)[2] - DIS
        if not sp is None and not pp is None:
            self._logger('===============================================')
            self._logger('===============================================')
            self._logger('mp = {}'.format(mp))

            if sp.position.z < min_z - DIS:
                sp = None
            if pp.position.z < min_z - DIS:
                pp = None

            vec = [mp[i-1] - mp[i] for i in range(len(mp))]
            vec = [v / np.linalg.norm(v) for v in vec]
            pland_d = [-1 * sum(v * p) for v, p in zip(vec, mp)]

            sp_min_d = DIS
            pp_min_d = DIS
            sp_min_d_vec = None
            pp_min_d_vec = None
            
            if sp != None:
                for v, d in zip(vec, pland_d):
                    sd = sum(np.array([sp.position.x, sp.position.y, sp.position.z]) * v) + d
                    if sd < sp_min_d:
                        sp_min_d = sd
                        sp_min_d_vec = v
                if sp_min_d < -1 * DIS:
                    sp = None
                elif sp_min_d < SUC_MIN_DIS:
                    sp.position.x = (SUC_MIN_DIS - sp_min_d) * sp_min_d_vec[0]
                    sp.position.y = (SUC_MIN_DIS - sp_min_d) * sp_min_d_vec[1]
                    sp.position.z = (SUC_MIN_DIS - sp_min_d) * sp_min_d_vec[2]
                s_factor = (sp_min_d + DIS) / (2 * DIS) if sp_min_d < DIS else 1
                if 'suc' in userdata.curr_tool:
                    s_factor /= 2**userdata.fail_cnt
                sq *= s_factor
                
            if pp != None:
                pp_min_d_tmp = 9999
                for v, d in zip(vec, pland_d):
                    pd = sum(np.array([pp.position.x, pp.position.y, pp.position.z]) * v) + d
                    if pd < pp_min_d:
                        pp_min_d = pd
                        pp_min_d_vec = v
                    elif pd < pp_min_d_tmp:
                        pp_min_d_tmp = pd
                        pp_min_d_vec = v
                if pp_min_d < -1 * DIS:
                    pp = None
                elif pp_min_d < PJ_MIN_DIS:
                    pp.position.x += (PJ_MIN_DIS - pp_min_d) * pp_min_d_vec[0]
                    pp.position.y += (PJ_MIN_DIS - pp_min_d) * pp_min_d_vec[1]
                    pp.position.z += (PJ_MIN_DIS - pp_min_d) * pp_min_d_vec[2]
                p_factor = (pp_min_d + DIS) / (2 * DIS) if pp_min_d < DIS else 1
                if 'pj' in userdata.curr_tool:
                    s_factor /= 2**userdata.fail_cnt
                pq *= p_factor



            # if not sp is None and not pp is None:
            #     vec = [mp[i-1] - mp[i] for i in range(len(mp))]
            #     vec = [v / np.linalg.norm(v) for v in vec]
            #     pland_d = [-1 * sum(v * p) for v, p in zip(vec, mp)]

            #     sp_min_d = DIS
            #     pp_min_d = DIS
            #     sp_min_d_vec = None
            #     pp_min_d_vec = None

            #     for v, d in zip(vec, pland_d):
            #         sd = sum(np.array([sp.position.x, sp.position.y, sp.position.z]) * v) + d
            #         pd = sum(np.array([pp.position.x, pp.position.y, pp.position.z]) * v) + d
            #         if sd < sp_min_d:
            #             sp_min_d = sd
            #             sp_min_d_vec = v
            #         if pd < pp_min_d:
            #             pp_min_d = pd
            #             pp_min_d_vec = v

            #     if sp_min_d < -1 * DIS:
            #         sp = None
            #     elif sp_min_d < 0:
            #         sp.position.x -= sp_min_d * sp_min_d_vec[0]
            #         sp.position.y -= sp_min_d * sp_min_d_vec[1]
            #         sp.position.z -= sp_min_d * sp_min_d_vec[2]
            #     if pp_min_d < -1 * DIS:
            #         pp = None
            #     elif pp_min_d < 0:
            #         pp.position.x -= pp_min_d * pp_min_d_vec[0]
            #         pp.position.y -= pp_min_d * pp_min_d_vec[1]
            #         pp.position.z -= pp_min_d * pp_min_d_vec[2]

            #     s_factor = (sp_min_d + DIS) / (2 * DIS) if sp_min_d < DIS else 1
            #     p_factor = (pp_min_d + DIS) / (2 * DIS) if pp_min_d < DIS else 1

            #     sq *= s_factor
            #     pq *= p_factor

        target_pose = PoseStamped()

        if not sp is None and not pp is None:
            if 'suc' in userdata.curr_tool:
                if sq > 0.5:
                    target_pose.pose = sp
                    tar_tool = 'suction'
                elif pq > sq * 2:
                    target_pose.pose = pp
                    tar_tool = 'pj'
                else:
                    target_pose.pose = sp
                    tar_tool = 'suction'

            elif 'pj' in userdata.curr_tool:
                if pq > 0.5:
                    target_pose.pose = pp
                    tar_tool = 'pj'
                elif sq > pq * 2:
                    target_pose.pose = sp
                    tar_tool = 'suction'
                else:
                    target_pose.pose = pp
                    tar_tool = 'pj'
            else:
                if sq * 2 > pq:
                    tar_tool = 'suction'
                else:
                    tar_tool = 'pj'
            
        else:
            if not sp is None:
                target_pose.pose = sp
                tar_tool = 'suction'
            elif not pp is None:
                target_pose.pose = pp
                tar_tool = 'pj'
            else:
                return 'failed'
            
        target_pose.header.frame_id = userdata.frame
            
        userdata.tar_tool = tar_tool
            
        self._logger('tar_tool is {}'.format(tar_tool))
        self._logger('===============================================')
        self._logger('===============================================')

        pose = target_pose.pose
        quat = qtn.quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        # y_trans = qtn.from_rotation_vector([0., np.pi, 0.])
        # quat *= y_trans

        quat *= qtn.quaternion(0.707106781186547, 0., 0.707106781186547, 0.)
        mat = qtn.as_rotation_matrix(quat)
        z_angle = acos(abs(mat[2][2]))
        if z_angle > 30 * np.pi / 180:
            a = z_angle - 30 * np.pi / 180
            v = np.cross(mat.transpose()[2], [0, 0, 1]) * a
            quat *= qtn.from_rotation_vector(v)

        pose.orientation.w = quat.w
        pose.orientation.x = quat.x
        pose.orientation.y = quat.y
        pose.orientation.z = quat.z

        if tar_tool == 'pj':
            pp = pose
            p = np.array([pp.position.x, pp.position.y, pp.position.z])
            q = qtn.quaternion(pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z)
            r = qtn.as_rotation_matrix(q)
            vy = r.transpose()[1]
            self._logger('vy = {}, pp_min_d_vec = {}, =============================================='.format(
                vy, pp_min_d_vec
            ))
            if np.dot(vy, pp_min_d_vec) < 0:
                q *= qtn.from_rotation_vector([0, 0, np.pi])
                pp.orientation.w = q.w
                pp.orientation.x = q.x
                pp.orientation.y = q.y
                pp.orientation.z = q.z
                r = qtn.as_rotation_matrix(q)

            t = np.identity(4)
            t[:3, :3] = r
            t[:3, 3] = p
            p0 = []
            p0.append(np.matmul(t, (np.array([ 0.01,  0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p0.append(np.matmul(t, (np.array([ 0.01, -0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p0.append(np.matmul(t, (np.array([-0.01, -0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p0.append(np.matmul(t, (np.array([-0.01,  0.005, 0.0, 1.0]) + np.array([0.0, 0.023, 0.0, 0.0])))[:3])

            p1 = []
            p1.append(np.matmul(t, (np.array([ 0.01,  0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p1.append(np.matmul(t, (np.array([ 0.01, -0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p1.append(np.matmul(t, (np.array([-0.01, -0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0])))[:3])
            p1.append(np.matmul(t, (np.array([-0.01,  0.005, 0.0, 1.0]) - np.array([0.0, 0.023, 0.0, 0.0])))[:3])



            depth_img = self.cv_gridge.imgmsg_to_cv2(userdata.img[1], desired_encoding='16UC1')
            camera_intrinsics = np.reshape(userdata.img_info[1].k, (3,3))
            avg_d0 = self.calculate_average_depth(depth_img, p0, camera_intrinsics, int(min_z * 1000)) / 1000.0
            avg_d1 = self.calculate_average_depth(depth_img, p1, camera_intrinsics, int(min_z * 1000)) / 1000.0

            if avg_d0 is None or avg_d1 is None:
                return 'failed'

            max_depth = min(avg_d0, avg_d1)
            if pp.position.z > max_depth:
                pp.position.z = max_depth
            target_pose.pose = pp
            self._logger('p0 is {}'.format(p0))
            self._logger('p1 is {}'.format(p1))
            self._logger('max_depth is {}'.format(max_depth))

        userdata.target_pose = target_pose
        return 'done'
    

    def calculate_average_depth(self, d_img, corners, camera_intrinsics, z_th):
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
        roi = d_img[min_y:max_y+1, min_x:max_x+1].flatten()
        roi_no_zero = []
        for x in roi:
            if x > z_th:
                roi_no_zero.append(x)
        
        # Calculate the average depth within the ROI
        if roi_no_zero == []:
            return None
        average_depth = np.mean(roi_no_zero)
        self._logger('max_depth is {}'.format(average_depth))
        
        return average_depth
