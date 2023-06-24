'''
Created on 26/02/2022

@author: Andy Chien
'''
from ur_msgs.msg import IOStates
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached

class GetDIOState(EventState):
    '''
    Get digital IO state

    -- io_service   string   Service name to set IO, ur default is /ur_hardware_interface/set_io
    -- sim          bool     Using simulator or not.

    ># pins         int[]    A list containing all to be controlled io pins.
    #> vals         int[]    A list storing the io pin state.

    <= done                  Set io finished.
    '''

    def __init__(self, io_topic, sim=False):
        '''
        Constructor
        '''
        super(GetDIOState, self).__init__(outcomes=['done'],
                                          input_keys=['pins'],
                                          output_keys=['vals'])
        self._sim = sim
        self._io_topic = io_topic
        if not sim:
            self._io_sub = ProxySubscriberCached({self._io_topic: IOStates})

    def execute(self, userdata):
        if self._sim:
            return 'done'

        if self._io_sub.has_msg(self._io_topic):
            msg = self._io_sub.get_last_msg(self._io_topic)
            userdata.vals = [msg.digital_in_states[pin].state for pin in userdata.pins]
            print('[GetDIOState]: userdata.vals = {}'.format(userdata.vals))
            return 'done'

    def on_enter(self, userdata):
        pass