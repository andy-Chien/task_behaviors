'''
Created on 03/05/2023
@author: Andy Chien
''' 
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from mr_msgs.srv import ComputeTrajectoryLength

class PlanningEvaluation(EventState):
    '''
    set initial robot collision objects to robot scene

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, terminal_rounds, do_evaluation, namespace=''):
        '''Constructor'''
        super(PlanningEvaluation, self).__init__(outcomes = ['done', 'finish'],
            input_keys = ['robot_trajectory', 'planning_time', 'planning_error_code'])
        self.success_rate = 0.0
        self.avg_planning_success_time = 0.0
        self.avg_joint_trajectory_length = 0.0
        self.avg_tool_trajectory_length = 0.0
        self.total_planning_success_time = 0.0
        self.total_joint_trajectory_length = 0.0
        self.total_tool_trajectory_length = 0.0
        self.success_count = 0.0
        self.planning_count = 0.0
        self.terminal = terminal_rounds
        self.do_evaluation = do_evaluation
        self._logger = self._node.get_logger()

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._compute_traj_service = '/' + namespace + '/compute_trajectory_length'
            self._target_link = namespace + '_tool_tip'
        else:
            self._compute_traj_service = '/compute_trajectory_length'
            self._target_link = 'tool_tip'

        self._traj_length_client = ProxyServiceCaller({self._compute_traj_service: ComputeTrajectoryLength})

    def execute(self, userdata):
        if self.do_evaluation and userdata.planning_error_code == MoveItErrorCodes.SUCCESS:
            if not self._traj_length_client.done(self._compute_traj_service):
                return
            result = self._traj_length_client.result(self._compute_traj_service)

            self.success_count += 1
            self.total_planning_success_time += userdata.planning_time
            self.total_joint_trajectory_length += result.joint_traj_length
            self.total_tool_trajectory_length += result.tool_traj_length

            self.avg_planning_success_time = self.total_planning_success_time / self.success_count
            self.avg_joint_trajectory_length = self.total_joint_trajectory_length / self.success_count
            self.avg_tool_trajectory_length = self.total_tool_trajectory_length / self.success_count
        
        if userdata.planning_error_code == MoveItErrorCodes.SUCCESS or \
            userdata.planning_error_code == MoveItErrorCodes.PLANNING_FAILED or \
            userdata.planning_error_code == MoveItErrorCodes.TIMED_OUT:
            self.planning_count += 1
        else:
            return 'done'
        
        if self.do_evaluation:
            self.success_rate = self.success_count / self.planning_count
            self._logger.info('===================================================================')
            self._logger.info('cnt: {}, s_rate: {}, p_time: {}, jt_len: {}, tt_len: {}'.format(
                self.planning_count, self.success_rate, self.avg_planning_success_time, \
                self.avg_joint_trajectory_length, self.avg_tool_trajectory_length))
            self._logger.info('===================================================================')

        return 'finish' if self.terminal and self.planning_count >= self.terminal else 'done'
    
    def on_enter(self, userdata):
        if self.do_evaluation and userdata.planning_error_code == MoveItErrorCodes.SUCCESS:
            self.call_compute_traj_length_service(userdata.robot_trajectory)
    
    def call_compute_traj_length_service(self, robot_trajectory):
        req = ComputeTrajectoryLength.Request()
        req.target_link = self._target_link
        req.trajectory = robot_trajectory.joint_trajectory
        self._traj_length_client.call_async(self._compute_traj_service, req)
        

        
        