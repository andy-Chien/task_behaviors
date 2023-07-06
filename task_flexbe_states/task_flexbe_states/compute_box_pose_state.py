#!/usr/bin/env python3
import os
from flexbe_core import EventState
import numpy as np
from math import atan2
import quaternion as qtn
from geometry_msgs.msg import Pose

'''
Created on 24.02.2022

@author: TaiTing Tsai
'''

class ComputeBoxPoseState(EventState):
    '''
    Get the grasp pose from GQCNN server.

    <= done 						          Get mask image.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ComputeBoxPoseState, self).__init__(outcomes=['done'],
                                                    input_keys=['marker_poses'],
                                                    output_keys=['box_pose'])
        

    def execute(self, userdata):
        '''
        Execute this state
        '''
        pos_list = []
        for p in userdata.marker_poses:
            pos = p.position
            pos_list.append(np.array([pos.x, pos.y, pos.z]))

        center_pos = np.mean(pos_list, axis=0)
        v1 = pos_list[-1][:2] - pos_list[0][:2]
        v2 = pos_list[1][:2] - pos_list[0][:2]
        v = v1 if np.linalg.norm(v1) < np.linalg.norm(v2) else v2
        q = qtn.from_rotation_vector(np.array([.0, .0, atan2(v[1], v[0])]))
        qz = qtn.quaternion(0., 1., 0., 0.)
        q *= qz

        pose = Pose()
        pose.position.x = center_pos[0]
        pose.position.y = center_pos[1]
        pose.position.z = center_pos[2]
        pose.orientation.w = q.w
        pose.orientation.x = q.x
        pose.orientation.y = q.y
        pose.orientation.z = q.z

        userdata.box_pose = pose
        return 'done'