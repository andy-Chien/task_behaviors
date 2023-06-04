'''
Created on 03/05/2023
@author: Andy Chien
''' 
import time
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from mr_msgs.srv import ComputeTrajectoryLength

PATH = "/home/andy/packing_ws/data/planner_data/"

class PlanningEvaluation(EventState):
    '''
    set initial robot collision objects to robot scene

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, terminal_rounds, do_evaluation, eval_rounds=0, namespace='', planner=''):
        '''Constructor'''
        super(PlanningEvaluation, self).__init__(outcomes = ['done', 'finish'],
            input_keys = ['robot_trajectory', 'planning_time', 'planning_error_code'])
        self.success_rate = 0.0
        self.avg_planning_success_time = 0.0
        self.avg_joint_trajectory_length = 0.0
        self.avg_tool_trajectory_length = 0.0
        self.avg_quat_trajectory_length = 0.0
        self.total_planning_success_time = 0.0
        self.total_joint_trajectory_length = 0.0
        self.total_tool_trajectory_length = 0.0
        self.total_quat_trajectory_length = 0.0
        self.planning_time = []
        self.tool_trajectory_length = []
        self.quat_trajectory_length = []
        self.joint_trajectory_length = []
        self.success_count = 0.0
        self.planning_count = 0.0
        self.terminal = terminal_rounds
        self.eval_rounds = eval_rounds
        self.do_evaluation = do_evaluation
        self.warm_up_cnt = 2
        self._logger = self._node.get_logger()
        self.last_plan_failed = False
        self.planner = planner
        self.namespace = namespace
        self.not_save_yet = True
        self.success_buffer = []
        self.r100_success_record = []

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._compute_traj_service = '/' + namespace + '/compute_trajectory_length'
            self._target_link = namespace + '_tool_tip'
        else:
            self._compute_traj_service = '/compute_trajectory_length'
            self._target_link = 'tool_tip'

        self._traj_length_client = ProxyServiceCaller({self._compute_traj_service: ComputeTrajectoryLength})

    def execute(self, userdata):
        if self.do_evaluation and userdata.planning_error_code == MoveItErrorCodes.SUCCESS \
                                                and self.planning_count > self.warm_up_cnt:
            if not self._traj_length_client.done(self._compute_traj_service):
                return
            result = self._traj_length_client.result(self._compute_traj_service)

            self.success_count += 1
            self.success_buffer.append(1.0)
            self.total_planning_success_time += userdata.planning_time
            self.total_joint_trajectory_length += result.joint_traj_length
            self.total_tool_trajectory_length += result.tool_traj_length
            self.total_quat_trajectory_length += result.quat_traj_length

            self.planning_time.append(userdata.planning_time)
            self.joint_trajectory_length.append(result.joint_traj_length)
            self.tool_trajectory_length.append(result.tool_traj_length)
            self.quat_trajectory_length.append(result.quat_traj_length)

            self.avg_planning_success_time = self.total_planning_success_time / self.success_count
            self.avg_joint_trajectory_length = self.total_joint_trajectory_length / self.success_count
            self.avg_tool_trajectory_length = self.total_tool_trajectory_length / self.success_count
            self.avg_quat_trajectory_length = self.total_quat_trajectory_length / self.success_count
        
        pec = userdata.planning_error_code
        if pec == MoveItErrorCodes.SUCCESS or \
            pec == MoveItErrorCodes.PLANNING_FAILED or \
            pec == MoveItErrorCodes.TIMED_OUT:
            if pec == MoveItErrorCodes.SUCCESS or not self.last_plan_failed:
                self.planning_count += 1
                if len(self.success_buffer) < self.planning_count:
                    self.success_buffer.append(0.0)
            self.last_plan_failed = pec != MoveItErrorCodes.SUCCESS
        else:
            return 'done'

        if self.do_evaluation and self.planning_count > self.warm_up_cnt and self.planning_count > 0:
            self.success_rate = self.success_count / self.planning_count
            if 0 < len(self.success_buffer) < 100:
                r100_success_rate = sum(self.success_buffer) / len(self.success_buffer)
            else:
                r100_success_rate = sum(self.success_buffer[-100 :]) / 100.0
            if self.planning_count % 100 < 1:
                self.r100_success_record.append(r100_success_rate)
            self._logger.info('===================================================================')
            self._logger.info('cnt: {}, s_rate: {}, r100_s_r: {}, p_time: {}, jt_len: {}, tt_len: {}, qt_len: {}'.format(
                self.planning_count, self.success_rate, r100_success_rate, self.avg_planning_success_time, \
                self.avg_joint_trajectory_length, self.avg_tool_trajectory_length, self.avg_quat_trajectory_length))
            self._logger.info('===================================================================')

        if self.warm_up_cnt > 0 and self.planning_count >= self.warm_up_cnt:
            self.warm_up_cnt = -1
            self.planning_count = 0.0
            self.success_buffer = []
            self.r100_success_record = []

        if self.planning_count >= self.eval_rounds and self.not_save_yet:
            self.not_save_yet = False
            name = self.planner + '_' + self.namespace
            with open(PATH + name + '.txt', 'w') as f:
                f.writelines(name +'_success_rate = ' + str(self.success_rate) +'\n')
                f.writelines(name +'_success_data = [')
                f.writelines([str(x) + ', ' for x in self.r100_success_record])
                f.writelines(']\n\n' + name + '_planning_time = ' + str(self.avg_planning_success_time) +'\n')
                f.writelines(name +'_planning_time_data = [')
                f.writelines([str(x) + ', ' for x in self.planning_time])
                f.writelines(']\n\n' + name + '_joint_trajectory_length: ' + str(self.avg_joint_trajectory_length) +'\n')
                f.writelines(name +'_joint_trajectory_length_data = [')
                f.writelines([str(x) + ', ' for x in self.joint_trajectory_length])
                f.writelines(']\n\n' + name + '_tool_trajectory_length: ' + str(self.avg_tool_trajectory_length) +'\n')
                f.writelines(name +'_tool_trajectory_length_data = [')
                f.writelines([str(x) + ', ' for x in self.tool_trajectory_length])
                f.writelines(']\n\n' + name + '_quat_trajectory_length: ' + str(self.avg_quat_trajectory_length) +'\n')
                f.writelines(name +'_quat_trajectory_length_data = [')
                f.writelines([str(x) + ', ' for x in self.quat_trajectory_length])
                f.writelines(']')
                f.close()

        return 'finish' if self.terminal and self.planning_count >= self.terminal else 'done'
    
    def on_enter(self, userdata):
        self.do_evaluation = self.do_evaluation and self.planning_count <= self.eval_rounds 
        if self.do_evaluation and userdata.planning_error_code == MoveItErrorCodes.SUCCESS \
                                                and self.planning_count > self.warm_up_cnt:
            self.call_compute_traj_length_service(userdata.robot_trajectory)
    
    def call_compute_traj_length_service(self, robot_trajectory):
        req = ComputeTrajectoryLength.Request()
        req.target_link = self._target_link
        req.trajectory = robot_trajectory.joint_trajectory
        self._traj_length_client.call_async(self._compute_traj_service, req)
        

        
        