'''
Created on 13.03.2023

@author: Andy Chien
'''
import copy
import rclpy
import numpy as np
import quaternion as qtn
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyTransformListener
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException



class ComputeFirstCapturePose(EventState):
    '''
    Set/create userdata value from input userdata. (userdata_src_names and userdata_dst_names must be same size.)

    -- painting_area 	double list[2][3]	A list containing position of two corner.

    '''

    def __init__(self, painting_area, distance_from_wall):
        '''
        Constructor
        '''
        super(ComputeFirstCapturePose, self).__init__(outcomes=['done'],
                                                      output_keys=['base_pose', 'arm_pose', 'mm_pose'])
        self._logger = ComputeFirstCapturePose._node.get_logger()
        self._painting_area = painting_area
        self._distance_from_wall = distance_from_wall
        ProxyTransformListener._initialize(EventState._node)
        self._tf_listener = ProxyTransformListener().listener()
        self._tf_buffer = self._tf_listener.buffer

    def execute(self, userdata):
        assert len(self._painting_area) == 2

        self._logger.info('self._painting_area = {}'.format(self._painting_area))
        p1, p2 = self._painting_area
        v_p1_p2 = np.array(p2) - np.array(p1)

        map_vz = np.array([0.0, 0.0, 1.0])

        camera_vy = -1 * map_vz
        camera_vz = np.cross(v_p1_p2, camera_vy)
        camera_vz /= np.linalg.norm(camera_vz)
        camera_vx = np.cross(camera_vy, camera_vz)
        camera_trans = np.identity(4)
        camera_trans[:3, :3] = np.array([camera_vx, camera_vy, camera_vz]).transpose()
        camera_trans[:3, 3] = np.array(p1) - camera_vz * self._distance_from_wall
        camera_quaternion = qtn.from_rotation_matrix(camera_trans[:3, :3])

        base_vx = camera_vz
        base_vy = -1 * camera_vx
        base_vz = map_vz
        base_trans = np.identity(4)
        base_trans[:3, :3] = np.array([base_vx, base_vy, base_vz]).transpose()
        base_trans[:3, 3] = camera_trans[:3, 3] - camera_vz * 0.5
        base_trans[2][3] = 0.0

        base_quaternion = qtn.from_rotation_matrix(base_trans[:3, :3])



        self._logger.info('base_trans = {}'.format(base_trans))

        try:
            t1 = self._tf_buffer.lookup_transform(
                "camera_link",
                "tool_tip",
                rclpy.time.Time()).transform
            t2 = self._tf_buffer.lookup_transform(
                "base_link",
                "mobile_base_footprint",
                rclpy.time.Time()).transform
        except TransformException as ex:
            self._logger.info(
                f'Could not transform map to mobile_base_footprint: {ex}')

        camera_t_tool = np.identity(4)
        arm_t_base = np.identity(4)
        camera_t_tool[0][3] = t1.translation.x
        camera_t_tool[1][3] = t1.translation.y
        camera_t_tool[2][3] = t1.translation.z
        
        arm_t_base[0][3] = t2.translation.x
        arm_t_base[1][3] = t2.translation.y
        arm_t_base[2][3] = t2.translation.z

        q1 = qtn.quaternion(t1.rotation.w, t1.rotation.x, t1.rotation.y, t1.rotation.z)
        q2 = qtn.quaternion(t2.rotation.w, t2.rotation.x, t2.rotation.y, t2.rotation.z)

        camera_t_tool[:3, :3] = qtn.as_rotation_matrix(q1)
        arm_t_base[:3, :3] = qtn.as_rotation_matrix(q2)


        arm_trans = np.array(np.mat(arm_t_base) * np.linalg.inv(np.mat(base_trans)) * \
                             np.mat(camera_trans) * np.mat(camera_t_tool))
        
        self._logger.info('arm_trans = {}'.format(arm_trans))

        arm_quaternion = qtn.from_rotation_matrix(arm_trans)

        
        userdata.base_pose = self.generate_pose_stamped(
            base_trans[:3, 3], base_quaternion, 'map')

        userdata.arm_pose = self.generate_pose_stamped(
            arm_trans[:3, 3], arm_quaternion, 'base_link')

        camera_trans[1][3] += 2.0
        camera_trans[2][3] -= 1.0
        next_pose = np.array(np.mat(camera_trans) * np.mat(camera_t_tool))
        arm_quaternion = qtn.from_rotation_matrix(next_pose)
        userdata.mm_pose = self.generate_pose_stamped(next_pose[:3, 3], arm_quaternion, 'map')

        return 'done'
    
    def generate_pose_stamped(self, pos, quat, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = quat.w
        pose.pose.orientation.x = quat.x
        pose.pose.orientation.y = quat.y
        pose.pose.orientation.z = quat.z
        return pose


        
