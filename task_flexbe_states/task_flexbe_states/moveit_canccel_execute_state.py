'''
Created on 26/02/2022

@author: Andy Chien
'''
from flexbe_core import EventState, Logger

class MoveitCancelExecuteState(EventState):
    '''
    Get digital IO state

    -- namespace    string
    -- sim          bool     Using simulator or not.

    ># exe_client   ProxyActionClient    action of execute_trajectory.

    <= done                  Set io finished.
    '''

    def __init__(self, namespace='', sim=False):
        '''
        Constructor
        '''
        super(MoveitCancelExecuteState, self).__init__(outcomes=['done'],
                                            input_keys=['exe_client'])
        
        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
        self._exe_action = '/' + namespace + '/execute_trajectory'

        self._sim = sim


    def execute(self, userdata):
        if self._sim:
            return 'done'
        Logger.loginfo('FFFFUUUUUUUCCCCCCCCKKKKKKKIIIINNNNNNNNGGGGGGGG SSSSSSSTTTTTTTTTTOOOOOOOOOPPPPPPPPPP')

        client = userdata.exe_client
        if client.is_active(self._exe_action):
            client.cancle(self._exe_action)
        elif client.has_result(self._exe_action):
            client.remove_result(self._exe_action)
        else:
            self._logger.warn('[MoveIt Waiting execution]: not active and no result!!')
        return 'done'
