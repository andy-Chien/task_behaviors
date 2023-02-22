import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory
from flexbe_core.proxy.proxy_action_client import ProxyActionClient
from flexbe_core import EventState

'''
Created on 21.02.2023

@author: Andy Chien
'''

class WaitForRunningState(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- namespace          string           robot name or namespace.

    ># exe_client         JointTrajectory  planned trajectory.
    
    <= waiting 						Robot is running.
    <= done 						Robot move done.
    <= collision 				    Robot during collision.
    <= failed 				        Robot move failed.
    '''


    def __init__(self, namespace=''):
        '''
        Constructor
        '''
        super(WaitForRunningState, self).__init__(outcomes=['waiting', 'done', 'collision', 'failed'],
                                                  input_keys=['exe_client'])
        if len(namespace) > 0 and not namespace.startswith('/'): 
            namespace = '/' + namespace
        self._exe_action = namespace + '/execute_trajectory'

        self._logger = self._node.get_logger()

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if userdata.exe_client is None or not userdata.exe_client.is_active(self._exe_action):
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!! userdata.exe_client is None !!!!!!!!!!!!!!!!!!!!!!")
            return 'done'

        if userdata.exe_client.has_result(self._exe_action):
            result = userdata.exe_client.get_result(self._exe_action)
            error_code = result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS or error_code == MoveItErrorCodes.PREEMPTED:
                return 'done'
            elif error_code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                self._logger.warn('[MoveIt Execute Trajectory State]: ' + str(error_code))
                return 'collision'
            else:
                self._logger.warn('[MoveIt Execute Trajectory State]: MoveItErrorCodes = {}'.format(error_code))
                return 'failed'
        else:
            self._logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! waiting !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            return 'waiting'