#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.condition_by_condition_state import ConditionByConditionState
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.get_random_pose_in_areas_state import GetRandomPoseInAreasState
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
from task_flexbe_states.planning_evaluation import PlanningEvaluation
from task_flexbe_states.set_data_by_data_state import SetDataByDataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 20 2023
@author: Andy Chien
'''
class SingleArmRandomTaskDemoSM(Behavior):
    '''
    single robot arm planning for the targert pose sampled from more than one area
    '''


    def __init__(self, node):
        super(SingleArmRandomTaskDemoSM, self).__init__()
        self.name = 'Single Arm Random Task Demo'

        # parameters of this behavior
        self.add_parameter('namespace', 'robot_1')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('random_areas', dict())
        self.add_parameter('planner_id', 'RRTConnectkConfigDefault')
        self.add_parameter('terminal_rounds', 100)
        self.add_parameter('do_evaluation', False)
        self.add_parameter('using_areas', dict())
        self.add_parameter('eval_rounds', 100)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ConditionByConditionState.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        GetRandomPoseInAreasState.initialize_ros(node)
        MoveItAsyncExecuteTrajectory.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        PlanningEvaluation.initialize_ros(node)
        SetDataByDataState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:675 y:527, x:183 y:548
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity'])
        _state_machine.userdata.velocity = 100
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.curr_area = 0
        _state_machine.userdata.running_cnt = 0

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:80 y:58
            OperatableStateMachine.add('get_curr_joints',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'get_random_joints', 'no_msg': 'get_curr_joints'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'start_joints'})

            # x:699 y:74
            OperatableStateMachine.add('async_execute',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'update_start_joints_and_area', 'failed': 'get_curr_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:724 y:340
            OperatableStateMachine.add('check_and_count_running',
                                        ConditionByConditionState(data_condition=lambda x: x+1, out_condition=lambda x: x['u'].running_cnt == x['condition_value'], condition_value=self.terminal_rounds, userdata_src_names=['running_cnt'], userdata_dst_names=['running_cnt']),
                                        transitions={'true': 'finished', 'false': 'plan_eval'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'running_cnt': 'running_cnt'})

            # x:226 y:186
            OperatableStateMachine.add('get_random_joints',
                                        GetRandomPoseInAreasState(group_name=self.group_name, joint_names=self.joint_names, areas=self.random_areas, using_areas=self.using_areas, namespace=self.namespace),
                                        transitions={'done': 'wait_for_running_until', 'failed': 'get_random_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'curr_area': 'curr_area', 'target_joints': 'target_joints', 'rand_area': 'rand_area'})

            # x:508 y:376
            OperatableStateMachine.add('plan_eval',
                                        PlanningEvaluation(terminal_rounds=self.terminal_rounds, do_evaluation=self.do_evaluation, eval_rounds=self.eval_rounds, namespace=self.namespace, planner=self.planner_id),
                                        transitions={'done': 'get_random_joints', 'finish': 'finished'},
                                        autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
                                        remapping={'robot_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:720 y:202
            OperatableStateMachine.add('update_start_joints_and_area',
                                        SetDataByDataState(userdata_src_names=['target_joints', 'rand_area'], userdata_dst_names=['start_joints', 'curr_area']),
                                        transitions={'done': 'check_and_count_running'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'target_joints': 'target_joints', 'rand_area': 'rand_area', 'start_joints': 'start_joints', 'curr_area': 'curr_area'})

            # x:520 y:193
            OperatableStateMachine.add('wait_for_running',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running', 'done': 'async_execute', 'collision': 'get_curr_joints', 'failed': 'get_curr_joints'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:219 y:291
            OperatableStateMachine.add('wait_for_running_until',
                                        WaitForRunningState(wait_until_complete_rate=50, wait_until_points_left=30, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running_until', 'done': 'Plan', 'collision': 'get_curr_joints', 'failed': 'get_curr_joints'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:216 y:383
            OperatableStateMachine.add('Plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=3, namespace=self.namespace, planner=self.planner_id, time_out=0.2, attempts=8),
                                        transitions={'failed': 'plan_eval', 'done': 'wait_for_running', 'retriable': 'Plan'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
