'''
Created on 04/19/2022
@author: Andy Chien
''' 
import numpy as np
from flexbe_core import EventState, Logger
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

class MoveItComputeIK(EventState):
    '''
    set initial robot collision objects to robot scene
    
    -- group_name       string       group name for moveit
    -- namespace        string       robot name or namespace.
    -- joint_names      string[]     joints name of robot

    ># start_joints     float[]      start position of joints for IK compute
    ># target_pose      multiple     can be geometry_msgs Pose PoseStamped or list of xyzwxyz
    ># translation_list float[]      translation list for target_pose x, y, z

    #> target_joints    float[]      target joints

    <= done                     set robot collision objects to initial pose success
    <= failed
    '''

    def __init__(self, group_name, joint_names, namespace='', from_frame='base_link', to_frame='tool_tip'):
        '''Constructor'''
        super(MoveItComputeIK, self).__init__(outcomes = ['done', 'failed'],
                                            input_keys = ['start_joints', 'target_pose','translation_list'],
                                            output_keys = ['target_joints'])
        self._node = MoveItComputeIK._node
        ProxyServiceCaller._initialize(self._node)

        self._logger = self._node.get_logger()

        self._req = GetPositionIK.Request()
        self._req.ik_request.group_name = group_name

        self._from_frame = from_frame

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._ik_service = '/' + namespace + '/compute_ik'
            self._joint_names = [namespace + '_' + jn for jn in joint_names]
            self._req.ik_request.ik_link_name = namespace + '_' + to_frame
        else:
            self._ik_service = '/compute_ik'
            self._joint_names = joint_names
            self._req.ik_request.ik_link_name = to_frame

        self._ik_client = ProxyServiceCaller({self._ik_service: GetPositionIK})

    def execute(self, userdata):
        if not self._ik_client.done(self._ik_service):
            self._logger.info('waiting ik')
            return
        result = self._ik_client.result(self._ik_service)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            userdata.target_joints = []
            for jn_ in self._joint_names:
                for jn, jp in zip(result.solution.joint_state.name, 
                                  result.solution.joint_state.position):
                    if jn == jn_:
                        userdata.target_joints.append(jp)
                        break
            self._logger.info('userdata.target_joints = {}'.format(userdata.target_joints))
            self._logger.info('userdata.joints name = {}'.format(self._joint_names))
            return 'done'
        else:
            self._logger.warn('Error code = {}'.format(result.error_code.val))
            return 'failed'

    def on_enter(self, userdata):
        self._req.ik_request.robot_state = \
            self.generate_robot_state(self._joint_names, userdata.start_joints)

        self._logger.warn('userdata.target_pose = {}'.format(userdata.target_pose))
        ps = PoseStamped()
        if isinstance(userdata.target_pose, PoseStamped):
            ps = userdata.target_pose
        elif isinstance(userdata.target_pose, Pose):
            ps.pose = userdata.target_pose
        elif isinstance(userdata.target_pose, (list, np.ndarray)):
            tp = userdata.target_pose
            assert len(tp) == 7
            ppp, ppo = ps.pose.position, ps.pose.orientation
            (ppp.x, ppp.y, ppp.z) = tp[:3]
            (ppo.w, ppo.x, ppo.y, ppo.z) = tp[3:]

        ps.header.stamp = self._node.get_clock().now().to_msg()
        if ps.header.frame_id == '':
            ps.header.frame_id = self._from_frame

        tl = userdata.translation_list
        if not tl is None and len(tl) == 3 and np.linalg.norm(tl) > 0.001:
            ps.pose.position.x += tl[0]
            ps.pose.position.y += tl[1]
            ps.pose.position.z += tl[2]
        self._logger.info('Pose of ik request is {}'.format(ps))

        self._req.ik_request.pose_stamped = ps
        self._ik_client.call_async(self._ik_service, self._req)

    def generate_robot_state(self, joint_names, start_joints):
        state = RobotState()
        if isinstance(start_joints, JointState):
            state.joint_state = start_joints
        else:
            sj = np.array(start_joints, dtype=float)
            if np.any(np.abs(sj) > np.pi * 2):
                sj = sj * np.pi / 180
            state.joint_state.name = joint_names
            state.joint_state.position = list(sj)

        state.joint_state.header.stamp = self._node.get_clock().now().to_msg()
        return state