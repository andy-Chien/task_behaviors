'''
Created on 26/02/2022

@author: Andy Chien
'''
import moveit_commander
from flexbe_core import EventState, Logger

class GraspedAndStop(EventState):
    '''
    Get digital IO state

    -- io_service   string   Service name to set IO, ur default is /ur_hardware_interface/set_io
    -- sim          bool     Using simulator or not.

    ># vals         int[]    A list storing the io pin state.

    <= done                  Set io finished.
    '''

    def __init__(self, robot_name, sim=False):
        '''
        Constructor
        '''
        super(GraspedAndStop, self).__init__(outcomes=['done'],
                                            input_keys=['vals'])

        self._group_name = robot_name
        self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
        self._sim = sim


    def execute(self, userdata):
        if self._sim:
            return 'done'

        if userdata.vals[0]:
            self._move_group.stop()
        return 'done'

    def on_enter(self, userdata):
        pass