'''
Created on 13.03.2023

@author: Andy Chien
'''
import numpy as np
import quaternion as qtn
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped



class PoseToBaseAndArm(EventState):
    '''
    Set/create userdata value from input userdata. (userdata_src_names and userdata_dst_names must be same size.)

    ># mm_pose      PoseStamped  mobile manipulator pose

    #> base_pose 	PoseStamped  base pose
    #> arm_pose 	PoseStamped  arm pose

    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(PoseToBaseAndArm, self).__init__(outcomes=['done'],
                                               input_keys=['mm_pose'],
                                               output_keys=['base_pose', 'arm_pose'])
        self._logger = PoseToBaseAndArm._node.get_logger()

    def execute(self, userdata):
        mp = userdata.mm_pose.pose
        mpq = mp.orientation
        mpp = mp.position

        self._logger.info('mpp = {}'.format(mpp))

        mpm = qtn.as_rotation_matrix(np.quaternion(mpq.w, mpq.x, mpq.y, mpq.z))
        mpvz = mpm[:, 2]

        vz = np.array([0.0, 0.0, 1.0])
        base_pose_vx = np.array([mpvz[0], mpvz[1], 0.0])
        base_pose_vx = base_pose_vx / np.linalg.norm(base_pose_vx)
        base_pose_vy = np.cross(vz, base_pose_vx)

        base_trans = np.array([base_pose_vx, base_pose_vy, vz]).transpose()
        base_pos = np.array([mpp.x, mpp.y, 0.5]) - base_pose_vx * 0.7
        self._logger.info('base_trans = {}'.format(base_trans))
        base_quaternion = qtn.from_rotation_matrix(base_trans)

        base_se3 = np.identity(4)
        base_se3[:3, :3] = base_trans
        base_se3[:3, 3] = base_pos
        
        userdata.base_pose = PoseStamped()
        userdata.base_pose.pose.position.x = base_pos[0]
        userdata.base_pose.pose.position.y = base_pos[1]
        userdata.base_pose.pose.position.z = 0.0
        userdata.base_pose.pose.orientation.w = base_quaternion.w
        userdata.base_pose.pose.orientation.x = base_quaternion.x
        userdata.base_pose.pose.orientation.y = base_quaternion.y
        userdata.base_pose.pose.orientation.z = base_quaternion.z
        userdata.base_pose.header.frame_id = 'map'

        self._logger.info('base_pos = {}'.format(base_pos))

        arm_se3 = np.identity(4)
        arm_se3[:3, :3] = qtn.as_rotation_matrix(np.quaternion(mpq.w, mpq.x, mpq.y, mpq.z))
        arm_se3[:3, 3] = np.array([mpp.x, mpp.y, mpp.z])

        base_t_tool = np.array(np.linalg.inv(np.mat(base_se3)) * np.mat(arm_se3))
        arm_quaternion = qtn.from_rotation_matrix(base_t_tool[:3, :3])
        arm_pos = base_t_tool[:3, 3]

        self._logger.info('base_t_tool = {}'.format(base_t_tool))

        userdata.arm_pose = PoseStamped()
        userdata.arm_pose.pose.position.x = arm_pos[0]
        userdata.arm_pose.pose.position.y = arm_pos[1]
        userdata.arm_pose.pose.position.z = arm_pos[2]
        userdata.arm_pose.pose.orientation.w = arm_quaternion.w
        userdata.arm_pose.pose.orientation.x = arm_quaternion.x
        userdata.arm_pose.pose.orientation.y = arm_quaternion.y
        userdata.arm_pose.pose.orientation.z = arm_quaternion.z
        userdata.arm_pose.header.frame_id = 'base_link'

        return 'done'


        
