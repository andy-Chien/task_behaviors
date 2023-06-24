#!/usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from task_msgs.srv import TaskPlanning


'''
Created on 13.03.2023

@author: Andy Chien
'''

class PaintingPathPlanning(EventState):
    '''
    convert base path to trajectory.

    -- namespace        string                  robot name or namespace.

    ># image_path       string                  painting img path
    #> painting_path 	nav_msgs/Path           painting path

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(PaintingPathPlanning, self).__init__(outcomes=['failed', 'done'],
                                                input_keys=['image_path'],
                                                output_keys=['painting_path'])

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._service = '/' + namespace + '/wall_painting_trajectory_planner/trigger'
        else:
            self._service = '/wall_painting_trajectory_planner/trigger'

        ProxyServiceCaller._initialize(EventState._node)
        self._logger = self._node.get_logger()
        self._client = ProxyServiceCaller({self._service: TaskPlanning})

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
            self._logger.error('Painting path plan failed!')
            return 'failed'

        userdata.painting_path = result.path
        return 'done'

    def on_enter(self, userdata):
        self._failed = False
        if self._client.is_available(self._service):
            req = TaskPlanning.Request()
            req.task_file_location = userdata.image_path
            self._client.call_async(self._service, req)
        else:
            self._failed = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)