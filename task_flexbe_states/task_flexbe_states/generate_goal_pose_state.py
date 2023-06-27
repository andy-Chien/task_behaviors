'''
Created on 06/21/2022
@author: TaiTing Tsai
''' 
import os
import sys   
import rclpy
import random
import numpy as np
import quaternion as qtn
from math import pi
from geometry_msgs.msg import PoseArray, Point, Quaternion
from flexbe_core import EventState, Logger
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class GenerateGoalPoseState(EventState):
    '''
    generate new goal pose by heared tf with translation or rotation
    
    -- Rotate           bool         group name for moveit
    -- Translate        bool         robot name or namespace.

    ># start_joints     float[]      initial joints value for IK compute.
    ># goal_pos         float[]      goal translation value for IK compute.
    ># goal_rot         float[]      goal rotation value for IK compute.

    #> target_joints                 target joint calculate by ik.

    <= done                          calculate goal pose success.
    '''

    def __init__(self, Rotate, Translate):
        '''Constructor'''
        super(GenerateGoalPoseState, self).__init__(outcomes = ['done'],
                                            input_keys = ['origin_pos', 'origin_rot', 'task_list'],
                                            output_keys = ['update_pos','update_rot'])
        self._node = GenerateGoalPoseState._node

        self._logger = self._node.get_logger()

        self.rotate = Rotate
        self.translate = Translate
        self.updated_pos = []
        self.updated_rot = []

    def execute(self, userdata):
        userdata.update_pos = self.updated_pos
        userdata.update_rot = [self.updated_rot.x,
                               self.updated_rot.y,
                               self.updated_rot.z,
                               self.updated_rot.w]
        return 'done'

    def on_enter(self, userdata):
        listened_rot_mat = qtn.as_rotation_matrix(np.quaternion(userdata.origin_rot[3],
                                                                userdata.origin_rot[0],
                                                                userdata.origin_rot[1],
                                                                userdata.origin_rot[2]))
        
        listened_trans_mat = np.array([[userdata.origin_pos[0]],
                                       [userdata.origin_pos[1]],
                                       [userdata.origin_pos[2]]])
        listened_transform_mat = np.append(listened_rot_mat, listened_trans_mat, axis=1)
        listened_transform_mat = np.append(listened_transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)
        if self.rotate:
            #w,x,y,z
            rot_mat = qtn.as_rotation_matrix(np.quaternion(userdata.task_list[3], 
                                                           userdata.task_list[0], 
                                                           userdata.task_list[1], 
                                                           userdata.task_list[2]))
            
            trans_mat = np.array([[0.0],
                                  [0.0],
                                  [0.0]])
            
            transform_mat = np.append(rot_mat, trans_mat, axis=1)
            transform_mat = np.append(transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)

            

            updated_trans_mat = np.matmul(listened_transform_mat, transform_mat)
        elif self.translate:
            #w,x,y,z
            rot_mat = np.eye(3)
            
            trans_mat = np.array([userdata.task_list[0],
                                  userdata.task_list[1],
                                  userdata.task_list[2]])
            
            transform_mat = np.append(rot_mat, trans_mat, axis=1)
            transform_mat = np.append(transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)

            
        else:
            rot_mat = np.eye(3)
            
            trans_mat = np.array([[0.0],
                                  [0.0],
                                  [0.0]])
            
            transform_mat = np.append(rot_mat, trans_mat, axis=1)
            transform_mat = np.append(transform_mat, np.array([[0., 0., 0., 1.]]), axis=0)

        updated_trans_mat = np.matmul(listened_transform_mat, transform_mat)

        self.updated_pos = [float(updated_trans_mat[0][3]),
                            float(updated_trans_mat[1][3]),
                            float(updated_trans_mat[2][3])]
        updated_trans_mat = np.delete(updated_trans_mat, 3, axis=0)
        updated_trans_mat = np.delete(updated_trans_mat, 3, axis=1)
        self.updated_rot = qtn.from_rotation_matrix(updated_trans_mat)
        