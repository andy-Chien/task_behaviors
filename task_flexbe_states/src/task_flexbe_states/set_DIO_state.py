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
    -- pins         int[]    A list containing all to be controlled io pins.
    -- vals         int[]    A list storing the io pin output.
    -- sim          bool     Using simulator or not.

    <= done                  Set io finished.
    <= failed       bool     Set io failed.
    '''

    def __init__(self, io_service, pins=[], vals=[], sim=False):
        '''
        Constructor
        '''
        super(SetDIOState, self).__init__(outcomes=['done', 'failed'])
        self._sim = sim
        self._io_service = io_service
        self._pins = pins
        self._vals = vals
        self._set_io = ProxyServiceCaller({self._io_service: SetIO})

    def execute(self, userdata):
        return 'done' if self.result else 'failed'

    def on_enter(self, userdata):
        if self._sim:
            self.result = True
        else:    
            for p, v in zip(userdata.pins, userdata.vals):
                self.result = self._set_io(SetIORequest.FUN_SET_DIGITAL_OUT, p, v)