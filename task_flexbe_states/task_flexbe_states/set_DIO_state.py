'''
Created on 26/02/2022

@author: Andy Chien
'''
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller

class SetDIOState(EventState):
    '''
    Set digital IO state

    -- io_service   string   Service name to set IO, ur default is /ur_hardware_interface/set_io
    -- sim          bool     Using simulator or not.

    ># pins         int[]    A list containing all to be controlled io pins.
    ># vals         int[]    A list storing the io pin output.

    <= done                  Set io finished.
    <= failed       bool     Set io failed.
    '''

    def __init__(self, io_service, namespace='', sim=False):
        '''
        Constructor
        '''
        super(SetDIOState, self).__init__(outcomes=['done', 'failed'],
                                          input_keys=['pins', 'vals'])
        self._sim = sim
        self._io_service = io_service

        if not io_service.startswith('/'): 
            io_service = '/' + io_service 

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._io_service = '/' + namespace + io_service
        else:
            self._io_service = namespace + io_service
        
        if not sim:
            self._set_io = ProxyServiceCaller({self._io_service: SetIO})

    def execute(self, userdata):
        return 'done' if self.result else 'failed'

    def on_enter(self, userdata):
        if self._sim:
            self.result = True
        else:    
            for p, v in zip(userdata.pins, userdata.vals):
                req = SetIORequest(SetIORequest.FUN_SET_DIGITAL_OUT, p, v)
                self.result = self._set_io.call(self._io_service, req)