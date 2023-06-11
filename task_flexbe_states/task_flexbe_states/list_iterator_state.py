'''
Created on 06/11/2023
@author: Andy Chien
''' 
from flexbe_core import EventState

class ListIteratorState(EventState):
    '''
    set initial robot collision objects to robot scene

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, the_list):
        '''Constructor'''
        super(ListIteratorState, self).__init__(outcomes = ['done', 'empty'],
                                            output_keys = ['data_out'])
        self.list = the_list
        self.it = 0

    def execute(self, userdata):
        if self.it >= len(self.list):
            return 'empty'
        userdata.data_out = self.list[self.it]
        self.it += 1
        return 'done'