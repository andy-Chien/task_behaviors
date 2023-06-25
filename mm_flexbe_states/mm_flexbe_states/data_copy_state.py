'''
Created on 04/19/2022
@author: Andy Chien
''' 
from flexbe_core import EventState

class MMDataCopyState(EventState):
    '''
    set initial robot collision objects to robot scene

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self):
        '''Constructor'''
        super(MMDataCopyState, self).__init__(outcomes = ['done'],
                                            input_keys = ['data_in'],
                                            output_keys = ['data_out'])

    def execute(self, userdata):
        userdata.data_out = userdata.data_in.copy()
        return 'done'
