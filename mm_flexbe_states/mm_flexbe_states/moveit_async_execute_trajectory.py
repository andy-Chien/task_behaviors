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

class MMMoveItAsyncExecuteTrajectory(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- group_name         string           move group name.
    -- namespace          string           robot name or namespace.

    ># joint_trajectory   JointTrajectory  planned trajectory.    

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    <= collision 				    Robot during collision.
    '''

    def __init__(self, group_name, namespace=''):
        '''
        Constructor
        '''
        super(MMMoveItAsyncExecuteTrajectory, self).__init__(outcomes=['done', 'failed'],
                                            input_keys=['joint_trajectory', 'exe_client'],
                                            output_keys=['exe_client'])
        self._group_name = group_name
        if len(namespace) > 0 and not namespace.startswith('/'): 
            namespace = '/' + namespace 
        self._exe_action = namespace + '/execute_trajectory'

        ProxyActionClient._initialize(EventState._node)
        self._logger = self._node.get_logger()
        self._exe_client_fail = False
        self._ns = namespace

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._exe_client_fail is True:
            return 'failed'

        if not userdata.exe_client.has_goal_response(self._exe_action):
            return
        else:
            goal_res = userdata.exe_client.get_goal_response(self._exe_action)
            self._logger.info("{}, goal_res = {}".format(self._ns, goal_res))
            if not goal_res.accepted:
                return 'failed'
        return 'done'

    def on_enter(self, userdata):
        if userdata.exe_client is None:
            userdata.exe_client = ProxyActionClient({self._exe_action: ExecuteTrajectory})

        if not userdata.exe_client.is_available(self._exe_action) or \
               userdata.exe_client.is_active(self._exe_action):
            self._exe_client_fail = True
        else:
            self._exe_client_fail = False

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = userdata.joint_trajectory
            userdata.exe_client.remove_goal_response(self._exe_action)
            userdata.exe_client.send_goal(self._exe_action, goal)

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)
