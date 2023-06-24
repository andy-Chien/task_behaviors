#!/usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from mm_msgs.srv import TrajMerge



'''
Created on 13.03.2023

@author: Andy Chien
'''

class MergeBaseArmTrajectory(EventState):
    '''
    convert base path to trajectory.

    -- namespace        string                  robot name or namespace.

    ># base_traj        trajectory_msgs/JointTrajectory  base trajectory
    ># arm_traj         trajectory_msgs/JointTrajectory  arm trajectory

    #> mm_trajectory 	trajectory_msgs/JointTrajectory  mobile manipulator trajectory

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(MergeBaseArmTrajectory, self).__init__(outcomes=['failed', 'done'],
                                                input_keys=['base_traj', 'arm_traj'],
                                                output_keys=['mm_trajectory'])

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._service = '/' + namespace + '/trajectory_merging'
        else:
            self._service = '/trajectory_merging'

        ProxyServiceCaller._initialize(EventState._node)
        self._logger = self._node.get_logger()

        self._client = ProxyServiceCaller({self._service: TrajMerge})

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self._failed:
            return 'failed'
        
        if not self._client.done(self._service):
            return
        
        result = self._client.result(self._service)
        if result.success:
            userdata.mm_trajectory = result.trajectory
            return 'done'
        else:
            return 'failed'

    def on_enter(self, userdata):
        self._failed = False
        
        if self._client.is_available(self._service):
            req = TrajMerge.Request()
            req.trajectories.append(userdata.base_traj)
            req.trajectories.append(userdata.arm_traj)
            self._client.call_async(self._service, req)
        else:
            self._failed = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)