#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mm_flexbe_states.moveit_compute_ik import MoveItComputeIK
from task_flexbe_states.decision_by_param_state import DecisionByParam
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 30 2023
@author: Andy Chien
'''
class MoveArmToPoseAsyncSM(Behavior):
    '''
    Move arm to target pose using MoveIt and mm trajectory controller.
In case of using current joint as start joints, just set the input start joints to any
    '''


    def __init__(self, node):
        super(MoveArmToPoseAsyncSM, self).__init__()
        self.name = 'Move Arm To Pose Async'

        # parameters of this behavior
        self.add_parameter('group_name', 'mobile_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('namespace', '')
        self.add_parameter('planner', 'BiTRRT')
        self.add_parameter('wait', True)
        self.add_parameter('use_curr_as_start', False)
        self.add_parameter('from_frame', 'base_link')
        self.add_parameter('to_frame', 'tool_tip')
        self.add_parameter('translation_in_target_frame', True)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DecisionByParam.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        MoveItAsyncExecuteTrajectory.initialize_ros(node)
        MoveItComputeIK.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:562 y:450, x:314 y:250
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose', 'translation_list', 'start_joints', 'velocity', 'exe_client', 'ik_target_frame'], output_keys=['target_joints', 'exe_client'])
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.target_pose = None
        _state_machine.userdata.translation_list = [0.0 ,0.0 ,0.0]
        _state_machine.userdata.planner = None
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.target_joints = None
        _state_machine.userdata.ik_target_frame = 'tool_tip'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:30 y:427
            OperatableStateMachine.add('need_current',
                                        DecisionByParam(decided=self.use_curr_as_start, outcomes=['True', 'False']),
                                        transitions={'True': 'wait_for_running_2', 'False': 'wait_until'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:500 y:169
            OperatableStateMachine.add('execute',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'wanna_wait', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:167 y:273
            OperatableStateMachine.add('get_curr',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'wait_until', 'no_msg': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'start_joints'})

            # x:94 y:45
            OperatableStateMachine.add('ik',
                                        MoveItComputeIK(group_name=self.group_name, joint_names=self.joint_names, namespace=self.namespace, from_frame=self.from_frame, to_frame=self.to_frame, translation_in_target_frame=self.translation_in_target_frame),
                                        transitions={'done': 'plan', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_pose': 'target_pose', 'target_frame': 'ik_target_frame', 'translation_list': 'translation_list', 'target_joints': 'target_joints'})

            # x:303 y:66
            OperatableStateMachine.add('plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=20, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'check_running_is_done', 'retriable': 'plan'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:359 y:415
            OperatableStateMachine.add('wait_for_running',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running', 'done': 'finished', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:172 y:416
            OperatableStateMachine.add('wait_for_running_2',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running_2', 'done': 'get_curr', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:86 y:146
            OperatableStateMachine.add('wait_until',
                                        WaitForRunningState(wait_until_complete_rate=70, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_until', 'done': 'ik', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:520 y:307
            OperatableStateMachine.add('wanna_wait',
                                        DecisionByParam(decided=self.wait, outcomes=['True', 'False']),
                                        transitions={'True': 'wait_for_running', 'False': 'finished'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:500 y:61
            OperatableStateMachine.add('check_running_is_done',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'check_running_is_done', 'done': 'execute', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
