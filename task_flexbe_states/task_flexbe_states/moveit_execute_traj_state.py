#!/usr/bin/env python
import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory
from flexbe_core.proxy.proxy_action_client import ProxyActionClient
from flexbe_core import EventState

'''
Created on 24.02.2022

@author: Andy Chien
'''

class MoveItExecuteTrajectoryState(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- group_name         string           move group name.
    -- namespace          string           robot name or namespace.

    ># joint_trajectory   JointTrajectory  planned trajectory.
    ># block_execute      bool             block in this state or not.
    

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    <= collision 				    Robot during collision.
    '''


    def __init__(self, group_name, namespace=''):
        '''
        Constructor
        '''
        super(MoveItExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed', 'collision', 'running'],
                                            input_keys=['joint_trajectory', 'block_execute'])
        self._group_name = group_name
        if len(namespace) > 0 and namespace[0] != '/': 
            namespace = '/' + namespace 
        self._exe_action = namespace + '/execute_trajectory'
        ProxyActionClient._initialize(EventState._node)
        self._exe_client = ProxyActionClient({self._exe_action: ExecuteTrajectory})
        self._running = False

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._exe_client.has_result(self._exe_action):
            result = self._exe_client.get_result(self._exe_action)
            error_code = result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS or error_code == MoveItErrorCodes.PREEMPTED:
                self._running = False
                return 'done'
            elif error_code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                rclpy.logwarn('[MoveIt Execute Trajectory State]: ' + str(error_code))
                self._running = False
                return 'collision'
            else:
                rclpy.logerr('[MoveIt Execute Trajectory State]: MoveItErrorCodes = {}'.format(error_code))
                self._running = False
                return 'failed'
        elif not userdata.block_execute:
            self._running = True
            return 'running'
        return

    def on_enter(self, userdata):
        if not self._running:
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = userdata.joint_trajectory
            self._exe_client.send_goal(self._exe_action, goal)

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)