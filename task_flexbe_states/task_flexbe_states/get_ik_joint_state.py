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

class GetIkJointState(EventState):
    '''
    calculate traget joints for robot from goal pose and rotation
    
    -- group_name       string       group name for moveit
    -- namespace        string       robot name or namespace.
    -- joint_names      string[]     joints name of robot

    ># start_joints     float[]      initial joints value for IK compute.
    ># goal_pos         float[]      goal translation value for IK compute.
    ># goal_rot         float[]      goal rotation value for IK compute.

    #> target_joints                 target joint calculate by ik.

    <= done                          calculate goal pose success.
    '''

    def __init__(self, group_name, joint_names, frame_id, namespace=''):
        '''Constructor'''
        super(GetIkJointState, self).__init__(outcomes = ['done', 'failed'],
                                            input_keys = ['start_joints','goal_pos','goal_rot'],
                                            output_keys = ['target_joints'])
        self._node = GetIkJointState._node
        ProxyServiceCaller._initialize(self._node)

        self._logger = self._node.get_logger()

        self._req = GetPositionIK.Request()
        self._req.ik_request.group_name = group_name

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)
        self.frame_id = frame_id
        self.listened_tf = False

        self._last_target_joint = None
        self._logger.info("group_name = {}".format(group_name))
        self._logger.info("joint_names = {}".format(joint_names))
        self._time_now = self._node.get_clock().now()

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._ik_service = '/' + namespace + '/compute_ik'
            self._joint_names = [namespace + '_' + jn for jn in joint_names]
            self._req.ik_request.ik_link_name = namespace + '_tool_tip'
        else:
            self._ik_service = '/compute_ik'
            self._joint_names = joint_names
            self._req.ik_request.ik_link_name = 'tool_tip'

        self._ik_client = ProxyServiceCaller({self._ik_service: GetPositionIK})

    def execute(self, userdata):
        if not self._ik_client.done(self._ik_service):
            self._logger.info('waiting ik')
            return
        result = self._ik_client.result(self._ik_service)
        
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.listened_tf == False
            userdata.target_joints = list(result.solution.joint_state.position)
            self._last_target_joint = userdata.target_joints
            self._logger.info('userdata.target_joints = {}'.format(userdata.target_joints))
            return 'done'
        elif self.listened_tf == False:
            return 'failed'
        else:
            self.listened_tf == False
            self._logger.warn('Error code = {}'.format(result.error_code.val))
            return 'failed'

    def on_enter(self, userdata):

        try:
            trans_tf = self.tf_buffer.lookup_transform(
                'base_link', 
                self.frame_id, 
                rclpy.time.Time())
            self._logger.info('-------------------------------{}'.format(trans_tf))
            goal_trans, goal_quat = self.calculate_tf_trans(trans_tf, userdata)
            self.listened_tf = True
        except TransformException as ex:
            self.transform_list = []
            Logger.logwarn('lookupTransform {}, {} failed!'.format('base_link', self.frame_id))
            self.listened_tf = False

        self._req.ik_request.robot_state = \
            self.generate_robot_state(self._joint_names, userdata.start_joints)

        self._logger.info('-------------poseeeeeeeeeeeeeeeeeeeeeeeeeee------------------{}'.format(userdata.goal_pos))
        self._logger.info('-------------poseeeeeeeeeeeeeeeeeeeeeeeeeee------------------{}'.format(userdata.goal_rot))
        self._req.ik_request.pose_stamped.header.stamp = self._time_now.to_msg()
        self._req.ik_request.pose_stamped.header.frame_id = 'world'
        self._req.ik_request.pose_stamped.pose.position.x = goal_trans[0]
        self._req.ik_request.pose_stamped.pose.position.y = goal_trans[1]
        self._req.ik_request.pose_stamped.pose.position.z = goal_trans[2]
        self._req.ik_request.pose_stamped.pose.orientation.x = goal_quat.x
        self._req.ik_request.pose_stamped.pose.orientation.y = goal_quat.y
        self._req.ik_request.pose_stamped.pose.orientation.z = goal_quat.z
        self._req.ik_request.pose_stamped.pose.orientation.w = goal_quat.w
        self._logger.info('-------------------------------{}'.format(self._req.ik_request.pose_stamped))

        self._ik_client.call_async(self._ik_service, self._req)

    def generate_robot_state(self, joint_names, start_joints):
        state = RobotState()
        self._time_now = self._node.get_clock().now()
        if type(start_joints) == JointState:
            state.joint_state = start_joints
        else:
            self._time_now = self._node.get_clock().now()
            sj = np.array(start_joints, dtype=float)
            if np.any(np.absolute(sj) > np.pi * 2):
                sj = sj * np.pi / 180
            state.joint_state.name = joint_names
            state.joint_state.position = list(sj)

        state.joint_state.header.stamp = self._time_now.to_msg()
        state.is_diff = True
        return state

    def calculate_tf_trans(self, listened_tf, user_trans):
        listened_rot_mat = qtn.as_rotation_matrix(np.quaternion(listened_tf.transform.rotation.w,
                                                                listened_tf.transform.rotation.x,
                                                                listened_tf.transform.rotation.y,
                                                                listened_tf.transform.rotation.z))
                
        listened_trans_mat = np.array([[listened_tf.transform.translation.x],
                                       [listened_tf.transform.translation.y],
                                       [listened_tf.transform.translation.z]])
        
        listened_mat = np.append(listened_rot_mat, listened_trans_mat, axis=1)
        listened_mat = np.append(listened_mat, np.array([[0., 0., 0., 1.]]), axis=0)
        
        user_rot_mat = qtn.as_rotation_matrix(np.quaternion(user_trans.goal_rot[3],
                                                            user_trans.goal_rot[0],
                                                            user_trans.goal_rot[1],
                                                            user_trans.goal_rot[2]))
                
        user_trans_mat = np.array([[user_trans.goal_pos[0]],
                                   [user_trans.goal_pos[1]],
                                   [user_trans.goal_pos[2]]])
        
        user_goal_mat = np.append(user_rot_mat, user_trans_mat, axis=1)
        user_goal_mat = np.append(user_goal_mat, np.array([[0., 0., 0., 1.]]), axis=0)

        real_goal_mat = np.matmul(listened_mat, user_goal_mat)
        self._logger.info('-------------------------------{}'.format(real_goal_mat))
        real_goal_trans = real_goal_mat[0:3, 3]
        real_goal_mat = np.delete(real_goal_mat, 3, axis=0)
        real_goal_mat = np.delete(real_goal_mat, 3, axis=1)
        

        real_quat = qtn.from_rotation_matrix(real_goal_mat)

        return real_goal_trans, real_quat