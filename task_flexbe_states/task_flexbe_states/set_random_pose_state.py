'''
Created on 04/19/2022
@author: Andy Chien
''' 
import os
import sys   
import rclpy
import numpy as np
import quaternion as qtn
from math import pi
from geometry_msgs.msg import PoseArray, Point, Quaternion
from flexbe_core import EventState, Logger
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from sensor_msgs.msg import JointState

class SetRandomPoseState(EventState):
    '''
    set initial robot collision objects to robot scene

    -- namespace        string                  robot name or namespace.
    -- joint_names      string[]                joints name of robot

    ># pos_min     float[]      min position of the bounding box for pose sample
    ># pos_max     float[]      max position of the bounding box for pose sample
    ># rot_min     float[]      min rotation [rx, ry, rz] for pose sample
    ># rot_max     float[]      max rotation [rx, ry, rz] for pose sample
    ># init_joints float[]      initial joints value for IK compute

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, group_name, joint_names, namespace=''):
        '''Constructor'''
        super(SetRandomPoseState, self).__init__(outcomes = ['done', 'failed'],
                                            input_keys = ['pos_min','pos_max', 'rot_min', 'rot_max', 'init_joints'],
                                            output_keys = ['sampled_joints'])
        self._node = SetRandomPoseState._node
        ProxyServiceCaller._initialize(self._node)
        self._logger = self._node.get_logger()

        self._req = GetPositionIK.Request()
        self._req.ik_request.group_name = group_name

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._ik_service = '/' + namespace + '/compute_ik'
            self._joint_names = [namespace + '_' + jn for jn in joint_names]
            self._req.ik_request.ik_link_name = namespace + '_tool0'
        else:
            self._ik_service = '/compute_ik'
            self._joint_names = joint_names
            self._req.ik_request.ik_link_name = 'tool0'

        self._ik_client = ProxyServiceCaller({self._ik_service: GetPositionIK})

    def execute(self, userdata):
        if not self._ik_client.done(self._ik_service):
            return
        result = self._ik_client.result(self._ik_service)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            userdata.sampled_joints = list(result.solution.joint_state.position)
            self._logger.info('userdata.sampled_joints = {}'.format(userdata.sampled_joints))
            return 'done'
        else:
            self._logger.warn('Error code = {}'.format(result.error_code.val))
            return 'failed'

    def on_enter(self, userdata):
        p_min, p_max, r_min, r_max = userdata.pos_min, userdata.pos_max, userdata.rot_min, userdata.rot_max
        rand_pos = np.random.uniform(low=p_min, high=p_max)
        rand_rot = np.random.uniform(low=r_min, high=r_max)
        rand_rot = rand_rot * np.pi / 180
        rand_qtn = qtn.from_euler_angles(rand_rot)
        print('rand_pos = {}, rand_rot = {}, rand_qtn = {}'.format(rand_pos, rand_rot, rand_qtn))
        # joint_names = self._js_sub.get_last_msg(self._js_topic).name
        self._req.ik_request.robot_state = \
            self.generate_robot_state(self._joint_names, userdata.init_joints)
        self._req.ik_request.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        self._req.ik_request.pose_stamped.pose.position.x = rand_pos[0]
        self._req.ik_request.pose_stamped.pose.position.y = rand_pos[1]
        self._req.ik_request.pose_stamped.pose.position.z = rand_pos[2]
        self._req.ik_request.pose_stamped.pose.orientation.x = rand_qtn.x
        self._req.ik_request.pose_stamped.pose.orientation.y = rand_qtn.y
        self._req.ik_request.pose_stamped.pose.orientation.z = rand_qtn.z
        self._req.ik_request.pose_stamped.pose.orientation.w = rand_qtn.w
        # self._req.ik_request.pose_stamped.pose.position = Point(0.5, 0.0, 0.04)
        # self._req.ik_request.pose_stamped.pose.orientation = Quaternion(0, 1, 0, 0)
        # self._result = self._ik_client.call(self._ik_service, self._req)
        self._ik_client.call_async(self._ik_service, self._req)

    def generate_robot_state(self, joint_names, start_joints):
        sj = np.array(start_joints, dtype=float)
        if np.any(np.absolute(sj) > np.pi * 2):
            sj = sj * np.pi / 180

        joint_state = JointState()
        joint_state.header.stamp = self._node.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.position = list(sj)
        state = RobotState()
        state.joint_state = joint_state
        return state