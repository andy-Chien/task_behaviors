#!/usr/bin/env python3
import rclpy
import numpy as np
import quaternion as qtn
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from tc5_msgs.srv import MappingPoint
from geometry_msgs.msg import PoseStamped, Point
from flexbe_core.proxy import ProxyTransformListener
from tf2_ros import TransformException


'''
Created on 13.03.2023

@author: Andy Chien
'''

class GetCapturePose(EventState):
    '''
    convert base path to trajectory.

    -- namespace        string                  robot name or namespace.

    ># image_path       string                  painting img path
    #> capture_pose 	nav_msgs/Path           painting path

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace='', init=False):
        '''
        Constructor
        '''
        super(GetCapturePose, self).__init__(outcomes=['failed', 'next', 'finish'],
                                                input_keys=['painting_area'],
                                                output_keys=['capture_pose'])

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._service = '/' + namespace + '/vision_preprocess_service_control'
        else:
            self._service = '/vision_preprocess_service_control'

        ProxyServiceCaller._initialize(EventState._node)
        self._logger = self._node.get_logger()
        self._client = ProxyServiceCaller({self._service: MappingPoint})
        self._is_first_capture = True

        ProxyTransformListener._initialize(EventState._node)
        self._tf_listener = ProxyTransformListener().listener()
        self._tf_buffer = self._tf_listener.buffer
        self._init = init

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._failed:
            return 'failed'
        
        if not self._client.done(self._service):
            return
        
        result = self._client.result(self._service)
        if not result.success:
            self._logger.error('GetCapturePose failed!')

        try:
            t1 = self._tf_buffer.lookup_transform(
                "camera_link",
                "tool_tip",
                rclpy.time.Time()).transform
        except TransformException as ex:
            self._logger.info(
                f'Could not transform map to mobile_base_footprint: {ex}')
            
        q1 = qtn.quaternion(t1.rotation.w, t1.rotation.x, t1.rotation.y, t1.rotation.z)
        camera_t_tool = np.identity(4)
        camera_t_tool[0][3] = t1.translation.x
        camera_t_tool[1][3] = t1.translation.y
        camera_t_tool[2][3] = t1.translation.z
        camera_t_tool[:3, :3] = qtn.as_rotation_matrix(q1)
        
        pose = result.nextpose

        self._logger.info('pose = {}'.format(pose))

        q2 = qtn.quaternion(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        map_t_camera = np.identity(4)
        map_t_camera[0][3] = pose.position.x
        map_t_camera[1][3] = pose.position.y
        map_t_camera[2][3] = pose.position.z
        map_t_camera[:3, :3] = qtn.as_rotation_matrix(q2)
        
        map_t_tool = np.array(np.mat(map_t_camera) * np.mat(camera_t_tool))
        map_q_tool = qtn.from_rotation_matrix(map_t_tool[:3, :3])

        pose_st = PoseStamped()
        pose_st.header.frame_id = 'map'
        pose_st.pose.position.x = map_t_tool[0][3]
        pose_st.pose.position.y = map_t_tool[1][3]
        pose_st.pose.position.z = map_t_tool[2][3]
        pose_st.pose.orientation.w = map_q_tool.w
        pose_st.pose.orientation.x = map_q_tool.x
        pose_st.pose.orientation.y = map_q_tool.y
        pose_st.pose.orientation.z = map_q_tool.z

        userdata.capture_pose = pose_st

        if result.finish:
            self._is_first_capture = True # reset for next painting area capture
            return 'finish'
        else:
            return 'next'

    def on_enter(self, userdata):
        self._failed = False
        if self._client.is_available(self._service):
            req = MappingPoint.Request()
            if self._init:
                req.ptlu.x = userdata.painting_area[0][0]
                req.ptlu.y = userdata.painting_area[0][1]
                req.ptlu.z = userdata.painting_area[0][2]

                req.ptbr.x = userdata.painting_area[1][0]
                req.ptbr.y = userdata.painting_area[1][1]
                req.ptbr.z = userdata.painting_area[1][2]
                req.flag = MappingPoint.Request.INITIAL

            elif self._is_first_capture:
                req.flag = MappingPoint.Request.START
                self._is_first_capture = False
            else:
                req.flag = MappingPoint.Request.CAPTURE

            self._client.call_async(self._service, req)
        else:
            self._failed = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)