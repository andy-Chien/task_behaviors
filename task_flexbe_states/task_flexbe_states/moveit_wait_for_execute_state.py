#!/usr/bin/env python
import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory
from flexbe_core.proxy.proxy_action_client import ProxyActionClient
from flexbe_core import EventState

'''
Created on 21.02.2023

@author: Andy Chien
'''

class WaitForRunningState(EventState):
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


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(WaitForRunningState, self).__init__(outcomes=['waiting', 'done', 'not_running'],
                                                  input_keys=['is_running', 'exe_client'])
        if len(namespace) > 0 and not namespace.startswith('/'): 
            namespace = '/' + namespace
        self._exe_action = namespace + '/execute_trajectory'

        # ProxyActionClient._initialize(EventState._node)
        self._logger = self._node.get_logger()


        # self._exe_client = ProxyActionClient({self._exe_action: ExecuteTrajectory})

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if not userdata.is_running or userdata.exe_client is None:
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! not_running !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return 'not_running'

        if userdata.exe_client.has_result(self._exe_action):
            return 'done'
        else:
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return 'waiting'