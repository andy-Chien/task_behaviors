'''
Created on 08/23/2021

@author: Frank Lyu
'''
from flexbe_core import EventState, Logger

class DecisionByParam(EventState):
    '''
    Set/create userdata value from input userdata use the condition function. (userdata_src_names and userdata_dst_names must be same size.)

    -- decided   	int or string  the sellected outcome
    -- outcomes         string[]       suggestions
    '''

    def __init__(self, decided, outcomes):
        '''
        Constructor
        '''
        super(DecisionByParam, self).__init__(outcomes=outcomes)
        self._decided = str(decided)

    def execute(self, _):
        return self._decided
    def on_enter(self, _):
        pass
