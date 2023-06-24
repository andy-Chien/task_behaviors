#!/usr/bin/env python3
from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient
from control_msgs.action import FollowJointTrajectory




'''
Created on 13.03.2023

@author: Andy Chien
'''

class ExecuteTrajectoryState(EventState):
    '''
    Execute trajectory using mm trajectory controller

    -- namespace        string                  robot name or namespace.

    ># trajectory 	trajectory_msgs/JointTrajectory  trajectory of arm, base, or both

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(ExecuteTrajectoryState, self).__init__(outcomes=['failed', 'done'],
                                                    input_keys=['trajectory'])

        self._logger = self._node.get_logger()

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._action = '/' + namespace + '/mm_trajectory_controller/follow_joint_trajectory'
        else:
            self._action = '/mm_trajectory_controller/follow_joint_trajectory'

        ProxyActionClient._initialize(EventState._node)

        self._client = ProxyActionClient({self._action: FollowJointTrajectory})

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
                self._logger.warn('Execute trajectory goal has been rejected')
                return 'failed'

        # Check if the action has been finished
        if not self._client.has_result(self._action):
            return

        result = self._client.get_result(self._action)
        self._client.remove_result(self._action)
        if result.error_code is FollowJointTrajectory.Result.SUCCESSFUL:
            self._logger.info('Trajectory execution succssful :)')
        else:
            self._logger.warn('Execute trajectory failed')
            return 'failed'

        return 'done'

    def on_enter(self, userdata):
        self._failed = False
        self._goal_accepted = False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = userdata.trajectory
        try:
            self._client.send_goal(self._action, goal)
        except Exception as e:
            self._failed = True
            self._logger.warn('Failed to send the compute path goal')

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)