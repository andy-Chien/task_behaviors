'''
Created on 04/19/2022
@author: Andy Chien
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

class GetRandomPoseInAreasState(EventState):
    '''
    set initial robot collision objects to robot scene
    
    -- group_name       string       group name for moveit
    -- namespace        string       robot name or namespace.
    -- areas            list(dict(str, float[]))  areas info include sets of pos_min, pos_max, rot_min, rot_max
    -- joint_names      string[]     joints name of robot

    ># init_joints      float[]      initial joints value for IK compute

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, group_name, joint_names, areas, using_areas, namespace=''):
        '''Constructor'''
        super(GetRandomPoseInAreasState, self).__init__(outcomes = ['done', 'failed'],
                                            input_keys = ['start_joints', 'curr_area'],
                                            output_keys = ['target_joints', 'rand_area'])
        self._node = GetRandomPoseInAreasState._node
        ProxyServiceCaller._initialize(self._node)

        self._logger = self._node.get_logger()

        self._req = GetPositionIK.Request()
        self._req.ik_request.group_name = group_name

        self._last_target_joint = None
        self._logger.info("group_name = {}".format(group_name))
        self._logger.info("joint_names = {}".format(joint_names))
        self._logger.info("areas = {}".format(areas))
        self._logger.info("using_areas = {}".format(using_areas))
        self._areas = [areas[indx] for indx in using_areas[namespace]]
        self._time_now = self._node.get_clock().now()

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._ik_service = '/' + namespace + '/compute_ik'
            self._joint_state_topic = '/' + namespace + '/joint_states'
            self._joint_names = [namespace + '_' + jn for jn in joint_names]
            self._req.ik_request.ik_link_name = namespace + '_tool0'
        else:
            self._ik_service = '/compute_ik'
            self._joint_state_topic = '/joint_states'
            self._joint_names = joint_names
            self._req.ik_request.ik_link_name = 'tool0'

        self._ik_client = ProxyServiceCaller({self._ik_service: GetPositionIK})
        self._joint_state_sub = ProxySubscriberCached({self._joint_state_topic: JointState})

    def execute(self, userdata):
        if not self._ik_client.done(self._ik_service):
            self._logger.info('waiting ik')
            return
        result = self._ik_client.result(self._ik_service)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            userdata.target_joints = list(result.solution.joint_state.position)
            self._last_target_joint = userdata.target_joints
            self._logger.info('userdata.target_joints = {}'.format(userdata.target_joints))
            return 'done'
        else:
            self._logger.warn('Error code = {}'.format(result.error_code.val))
            return 'failed'

    def on_enter(self, userdata):
        if len(self._areas) > 2:
            rand_area = random.randint(0, len(self._areas) - 1)
            if rand_area == userdata.curr_area:
                self._logger.info('rand_area == self._curr_area')
                return self.on_enter(userdata)
        else:
            rand_area = (userdata.curr_area + 1) % len(self._areas)
        userdata.rand_area = rand_area

        area_indx = random.randint(0, len(self._areas[rand_area]) - 1)

        self._time_now = self._node.get_clock().now()
        
        self._logger.info("self._areas = {}, {}\n{}".format(rand_area, len(self._areas), self._areas))
        rand_pos = np.random.uniform(
            low=self._areas[rand_area][area_indx]['pos_min'],
            high=self._areas[rand_area][area_indx]['pos_max']
        )
        rand_rot = np.random.uniform(
            low=self._areas[rand_area][area_indx]['rot_min'],
            high=self._areas[rand_area][area_indx]['rot_max']
        )
        rand_rot = rand_rot * np.pi / 180
        rand_qtn = qtn.from_euler_angles(rand_rot)

        self._req.ik_request.robot_state = \
            self.generate_robot_state(self._joint_names, userdata.start_joints)

        self._req.ik_request.pose_stamped.header.stamp = self._time_now.to_msg()
        self._req.ik_request.pose_stamped.pose.position.x = rand_pos[0]
        self._req.ik_request.pose_stamped.pose.position.y = rand_pos[1]
        self._req.ik_request.pose_stamped.pose.position.z = rand_pos[2]
        self._req.ik_request.pose_stamped.pose.orientation.x = rand_qtn.x
        self._req.ik_request.pose_stamped.pose.orientation.y = rand_qtn.y
        self._req.ik_request.pose_stamped.pose.orientation.z = rand_qtn.z
        self._req.ik_request.pose_stamped.pose.orientation.w = rand_qtn.w

        self._ik_client.call_async(self._ik_service, self._req)

    def generate_robot_state(self, joint_names, start_joints):
        state = RobotState()
        if type(start_joints) == JointState:
            state.joint_state = start_joints
        else:
            sj = np.array(start_joints, dtype=float)
            if np.any(np.absolute(sj) > np.pi * 2):
                sj = sj * np.pi / 180
            state.joint_state.name = joint_names
            state.joint_state.position = list(sj)

        state.joint_state.header.stamp = self._time_now.to_msg()
        return state