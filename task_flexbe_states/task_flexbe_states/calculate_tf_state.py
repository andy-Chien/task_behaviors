#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
import rclpy
import numpy as np
import quaternion as qtn
from geometry_msgs.msg import Pose, TransformStamped
'''
Created on 23.06.2023

@author: TaiTing Tsai
'''

class CalculateTfState(EventState):
    '''
    Calculate tf with transform matrix.

    -- tf_right_muti_count    int      The number of right multiply matrix.
    -- tf_left_muti_count     int      The number of left multiply matrix.
    -- static_axis            string   The axis which remain the same.

    ># transform                       Transformation pose for tf calculation.
    ># listened_transform              Transformation pose listened from tf, could be list.
    
    #> updated_transform               New transform for updating
    
    <= done 					       Finished the new tf calculation.
    <= failed 					       Failed the new tf calculation.
    '''

    def __init__(self, tf_right_muti_count, tf_left_muti_count, static_axis=''):
        '''
        Constructor
        '''
        super(CalculateTfState, self).__init__(outcomes=['done', 'failed', 'listen_tf'],
                                               input_keys=['transform', 'listened_transform'],
                                               output_keys=['updated_transform'])
        

        self._node = CalculateTfState._node
        self._logger = self._node.get_logger().info
        self.static_axis = static_axis
        self.tf_right_muti_count = tf_right_muti_count
        self.tf_left_muti_count = tf_left_muti_count
        self.listened_trans_mat = []
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if len(userdata.listened_transform) == 0:
            return "listen_tf"
        
        else:
            rot_mat = qtn.as_rotation_matrix(np.quaternion(userdata.transform.orientation.w, 
                                                           userdata.transform.orientation.x, 
                                                           userdata.transform.orientation.y, 
                                                           userdata.transform.orientation.z))
            trans_mat = np.array([[userdata.transform.position.x],
                                  [userdata.transform.position.y],
                                  [userdata.transform.position.z]])
            
            transform_mat = np.append(rot_mat, trans_mat, axis=1)
            transform_mat = np.append(transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)
            transform_mat = np.linalg.inv(transform_mat)

            
            
            for i in range(len(userdata.listened_transform)):
                listened_rot_mat = qtn.as_rotation_matrix(np.quaternion(userdata.listened_transform[i].transform.rotation.w,
                                                                        userdata.listened_transform[i].transform.rotation.x,
                                                                        userdata.listened_transform[i].transform.rotation.y,
                                                                        userdata.listened_transform[i].transform.rotation.z))
                
                listened_trans_mat = np.array([[userdata.listened_transform[i].transform.translation.x],
                                               [userdata.listened_transform[i].transform.translation.y],
                                               [userdata.listened_transform[i].transform.translation.z]])
                
                listened_transform_mat = np.append(listened_rot_mat, listened_trans_mat, axis=1)
                listened_transform_mat = np.append(listened_transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)
                self.listened_trans_mat.append(listened_transform_mat)
            for i in range(self.tf_left_muti_count):
                if i == 0:
                    left_muti_mat = self.listened_trans_mat[i]
                else:
                    left_muti_mat = np.matmul(left_muti_mat, self.listened_trans_mat[i])

            for k in range(self.tf_left_muti_count, len(self.listened_trans_mat)):
                if k == self.tf_left_muti_count:
                    right_muti_mat = self.listened_trans_mat[k]
                else:
                    right_muti_mat = np.matmul(right_muti_mat, self.listened_trans_mat[k])

            updated_trans_mat = np.matmul(left_muti_mat, transform_mat)
            updated_trans_mat = np.matmul(updated_trans_mat, right_muti_mat)
            if self.static_axis != '':
                idx = lambda key: 0 if 'x' in key else 1 if 'y' in key else 2
                m = updated_trans_mat[:3, :3].T
                for i, v in enumerate(m):
                    if i == idx:
                        for j in range(3): v[j] = 1 if j == idx else 0
                        continue
                    d = np.linalg.norm([x if j!=idx else 0 for j, x in enumerate(v)])
                    for j in range(3):
                        v[j] = v[j] / d if j!=idx else 0
                updated_trans_mat[:3, :3] = m.T
            userdata.updated_transform = updated_trans_mat
            return 'done'


    def on_enter(self, userdata):
        self.listened_trans_mat = []
    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)
