'''
Created on 03/05/2023
@author: Andy Chien
''' 
from flexbe_core import EventState
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory

class PlanningEvaluation(EventState):
    '''
    set initial robot collision objects to robot scene

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, finish_count, do_evaluation):
        '''Constructor'''
        super(PlanningEvaluation, self).__init__(outcomes = ['done', 'finish'],
            input_keys = ['joint_trajectory', 'planning_time', 'planning_error_code'])
        self.success_rate = 0.0
        self.avg_planning_success_time = 0.0
        self.avg_joint_trajectory_length = 0.0
        self.avg_tool_trajectory_length = 0.0
        self.total_planning_success_time = 0.0
        self.total_joint_trajectory_length = 0.0
        self.total_tool_trajectory_length = 0.0
        self.success_count = 0.0
        self.planning_count = 0.0
        self.finish_count = finish_count
        self.do_evaluation = do_evaluation

    def execute(self, userdata):
        if not self.do_evaluation:
            return 'done'

        if userdata.planning_error_code == MoveItErrorCodes.SUCCESS:
            self.success_count += 1
            self.planning_count += 1
            self.total_planning_success_time += userdata.planning_time
            self.total_joint_trajectory_length += self.calc_joint_traj_length(userdata.joint_trajectory)
            self.total_tool_trajectory_length += self.calc_tool_traj_length(userdata.joint_trajectory)

            self.avg_planning_success_time = self.total_planning_success_time / self.success_count
            self.avg_joint_trajectory_length = self.total_joint_trajectory_length / self.success_count
            self.avg_tool_trajectory_length = self.total_tool_trajectory_length / self.success_count


        elif userdata.planning_error_code == MoveItErrorCodes.PLANNING_FAILED or \
             userdata.planning_error_code == MoveItErrorCodes.TIMED_OUT:
            self.planning_count += 1
            print('planning time when failed = {}'.format(userdata.planning_time))

        else:
            return 'done'
        
        self.success_rate = self.success_count / self.planning_count
        
        print('===================================================================================')
        print('cnt: {}, success_rate: {}, planning_time: {}, jt_length: {}, tt_length: {}'.format(
            self.planning_count, self.success_rate, self.avg_planning_success_time, \
            self.avg_joint_trajectory_length, self.avg_tool_trajectory_length))
        print('===================================================================================')

        return 'finish' if self.finish_count and self.planning_count >= self.finish_count else 'done'
    
    def calc_joint_traj_length(self, traj):
        return 0
    
    def calc_tool_traj_length(self, traj):
        return 0
        

        
        