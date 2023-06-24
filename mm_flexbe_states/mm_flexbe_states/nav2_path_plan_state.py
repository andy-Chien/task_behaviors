#!/usr/bin/env python3
import rclpy
import numpy as np
import quaternion as qtn
from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient, ProxyTransformListener
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException


'''
Created on 13.03.2023

@author: Andy Chien
'''

class Nav2PathPlanState(EventState):
    '''
    Uses Nav2 to plan the path of specified target pose.

    -- namespace        string                  robot name or namespace.
    -- planner_id       string                  planner name.

    ># target_pose 	    PoseStamped	   Target pose of mobile base in map frame.

    #> base_path        nav_msgs::msg::Path  planned path

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace='', planner_id='GridBased'):
        '''
        Constructor
        '''
        super(Nav2PathPlanState, self).__init__(outcomes=['failed', 'done'],
                                                input_keys=['target_pose'],
                                                output_keys=['base_path'])

        self._logger = self._node.get_logger()

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._action = '/' + namespace + '/compute_path_to_pose'
        else:
            self._action = '/compute_path_to_pose'

        ProxyActionClient._initialize(EventState._node)
        ProxyTransformListener._initialize(EventState._node)
        self._tf_listener = ProxyTransformListener().listener()
        self._tf_buffer = self._tf_listener.buffer
        self._client = ProxyActionClient({self._action: ComputePathToPose})

        self._goal = ComputePathToPose.Goal()
        self._goal.use_start = False
        self._goal.planner_id = planner_id
        self._failed = False
        self._goal_accepted = False

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._failed:
            return 'failed'
        
        if not self._goal_accepted and not self._client.has_goal_response(self._action):
            return
        if not self._goal_accepted:
            self._goal_accepted = self._client.get_goal_response(self._action).accepted
            self._client.remove_goal_response(self._action)
            if not self._goal_accepted:
                self._logger.warn('Compute path goal has been rejected')
                return 'failed'

        # Check if the action has been finished
        if not self._client.has_result(self._action):
            return

        result = self._client.get_result(self._action)
        if len(result.path.poses) < 1:
            self._logger.warn('Compute path failed')
            return 'failed'

        userdata.base_path = result.path
        start_pose = PoseStamped()
        try:
            t = self._tf_buffer.lookup_transform(
                "map",
                "mobile_base_footprint",
                rclpy.time.Time())
        except TransformException as ex:
            self._logger.info(
                f'Could not transform map to mobile_base_footprint: {ex}')
            
        start_pose.header = result.path.poses[0].header
        start_pose.pose.position.x = t.transform.translation.x
        start_pose.pose.position.y = t.transform.translation.y
        start_pose.pose.position.z = t.transform.translation.z
        start_pose.pose.orientation = t.transform.rotation
        userdata.base_path.poses.insert(0, start_pose)

        p_0 = userdata.base_path.poses[0].pose
        p_end = userdata.base_path.poses[-1].pose
        q_0 = qtn.quaternion(
            p_0.orientation.w, p_0.orientation.x, p_0.orientation.y, p_0.orientation.z)
        q_end = qtn.quaternion(
            p_end.orientation.w, p_end.orientation.x, p_end.orientation.y, p_end.orientation.z)
        pose_size = len(userdata.base_path.poses)
        for i, pose in enumerate(userdata.base_path.poses):
            q = qtn.slerp(q_0, q_end, 0.0, 1.0, float(i) / float(pose_size - 1))
            pose.pose.orientation.w = q.w
            pose.pose.orientation.x = q.x
            pose.pose.orientation.y = q.y
            pose.pose.orientation.z = q.z
        return 'done'

    def on_enter(self, userdata):
        self._failed = False
        self._goal_accepted = False
        self._goal.goal = userdata.target_pose
        self._logger.info('userdata.target_pose = {}'.format(userdata.target_pose))
        try:
            self._client.send_goal(self._action, self._goal)
        except Exception as e:
            self._failed = True
            self._logger.warn('Failed to send the compute path goal')

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)