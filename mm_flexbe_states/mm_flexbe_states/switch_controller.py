#!/usr/bin/env python3
from rclpy.duration import Duration
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from controller_manager_msgs.srv import SwitchController

'''
Created on 13.03.2023

@author: Andy Chien
'''

class SwitchControllers(EventState):
    '''
    convert base path to trajectory.

    -- namespace        string                  robot name or namespace.

    ># image_path       string                  painting img path
    #> painting_path 	nav_msgs/Path           painting path

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, active_controllers, deactive_controllers, namespace=''):
        '''
        Constructor
        '''
        super(SwitchControllers, self).__init__(outcomes=['failed', 'done'])

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._service = '/' + namespace + '/controller_manager/switch_controller'
        else:
            self._service = '/controller_manager/switch_controller'

        self._active_controllers = active_controllers
        self._deactive_controllers = deactive_controllers


        ProxyServiceCaller._initialize(EventState._node)
        self._logger = self._node.get_logger()
        self._client = ProxyServiceCaller({self._service: SwitchController})

    def execute(self, _):
        '''
        Execute this state
        '''
        if self._failed:
            return 'failed'
        
        if not self._client.done(self._service):
            return
        
        result = self._client.result(self._service)
        if not result.ok:
            self._logger.error('swicth controller failed!')
            return 'failed'

        return 'done'

    def on_enter(self, _):
        self._failed = False
        if self._client.is_available(self._service):
            req = SwitchController.Request()
            req.activate_controllers = self._active_controllers
            req.deactivate_controllers = self._deactive_controllers
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.activate_asap = True
            req.timeout = Duration(seconds=5).to_msg()
            self._client.call_async(self._service, req)
        else:
            self._failed = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)