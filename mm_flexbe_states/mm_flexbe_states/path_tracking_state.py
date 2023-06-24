#!/usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from mm_msgs.srv import PathTracking


'''
Created on 13.03.2023

@author: Andy Chien
'''

class PathTrackingState(EventState):
    '''
    convert base path to trajectory.

    -- namespace        string                  robot name or namespace.

    ># painting_path    nav_msgs/Path           painting path

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(PathTrackingState, self).__init__(outcomes=['failed', 'done'],
                                           input_keys=['painting_path'])

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._service = '/' + namespace + '/path_ik_service'
        else:
            self._service = '/path_ik_service'

        ProxyServiceCaller._initialize(EventState._node)
        self._logger = self._node.get_logger()
        self._client = ProxyServiceCaller({self._service: PathTracking})

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
            self._logger.error('Path tracking failed!')
            return 'failed'

        return 'done'

    def on_enter(self, userdata):
        self._failed = False
        if self._client.is_available(self._service):
            req = PathTracking.Request()
            req.path = userdata.painting_path
            self._client.call_async(self._service, req)
        else:
            self._failed = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)