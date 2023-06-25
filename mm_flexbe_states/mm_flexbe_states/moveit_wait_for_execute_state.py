import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import ExecuteTrajectory
from flexbe_core.proxy.proxy_action_client import ProxyActionClient
from flexbe_core import EventState

'''
Created on 21.02.2023

@author: Andy Chien
'''

class MMWaitForRunningState(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- namespace          string           robot name or namespace.

    ># exe_client         JointTrajectory  planned trajectory.
    
    <= waiting 						Robot is running.
    <= done 						Robot move done.
    <= collision 				    Robot during collision.
    <= failed 				        Robot move failed.
    '''


    def __init__(self, wait_until_complete_rate=0, wait_until_points_left=0, namespace=''):
        '''
        Constructor
        '''
        super(MMWaitForRunningState, self).__init__(outcomes=['waiting', 'done', 'collision', 'failed'],
                                                  input_keys=['exe_client'])
        if len(namespace) > 0 and not namespace.startswith('/'): 
            namespace = '/' + namespace
        self._exe_action = namespace + '/execute_trajectory'
        self._wait_until_complete_rate = wait_until_complete_rate
        self._wait_until_points_left = wait_until_points_left
        self._logger = self._node.get_logger()

    def execute(self, userdata):
        '''
        Execute this state
        '''
        if userdata.exe_client is None:
            return 'done'

        client = userdata.exe_client

        if client.is_active(self._exe_action):
            complete_rate = 0
            points_left = 999999
            if client.has_feedback(self._exe_action):
                fb = client.get_feedback(self._exe_action).feedback
                client.remove_feedback(self._exe_action)
                fb_str_list = fb.state.split(' ')
                if fb_str_list[0] == 'RUNNING':
                    try:
                        complete_rate = int(fb_str_list[1])
                        points_left = int(fb_str_list[4])
                    except:
                        self._logger.error("fb_str_list[1] can't be convert to integer")
            self._logger.info("!!!!!!!!!!!!! waiting {} / 100, {} points left !!!!!!!!!!!!!".format(
                complete_rate, points_left))
            keep_wating = not (self._wait_until_complete_rate or self._wait_until_points_left)
            if self._wait_until_complete_rate and complete_rate < self._wait_until_complete_rate:
                keep_wating = True
            if self._wait_until_points_left and points_left > self._wait_until_points_left:
                keep_wating = True
            return 'waiting' if keep_wating else 'done'

        elif client.has_result(self._exe_action):
            result = client.get_result(self._exe_action)
            error_code = result.error_code.val
            client.remove_result(self._exe_action)
            if error_code == MoveItErrorCodes.SUCCESS or error_code == MoveItErrorCodes.PREEMPTED:
                return 'done'
            elif error_code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
                self._logger.warn('[MoveIt Waiting execution: ErrorCodes = {}'.format(error_code))
                return 'collision'
            else:
                self._logger.warn('[MoveIt Waiting execution: ErrorCodes = {}'.format(error_code))
                return 'failed'
        else:
            self._logger.warn('[MoveIt Waiting execution]: not active and no result!!')
            return 'done'
