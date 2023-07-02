#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
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
class MoveArmToJointsAsyncSM(Behavior):
    '''
    Move arm to target joints using MoveIt and mm trajectory controller.
In case of using current joint as start joints, just set the input start joints to any
    '''


    def __init__(self, node):
        super(MoveArmToJointsAsyncSM, self).__init__()
        self.name = 'Move Arm To Joints Async'

        # parameters of this behavior
        self.add_parameter('group_name', 'mobile_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('namespace', '')
        self.add_parameter('planner', 'BiTRRT')
        self.add_parameter('wait', True)
        self.add_parameter('use_curr_as_start', False)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DecisionByParam.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        MoveItAsyncExecuteTrajectory.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:562 y:450, x:314 y:219
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_joints', 'velocity', 'target_joints', 'exe_client'], output_keys=['target_joints'])
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.planner = None
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.target_joints = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:95 y:398
            OperatableStateMachine.add('need_current',
                                        DecisionByParam(decided=self.use_curr_as_start, outcomes=['True', 'False']),
                                        transitions={'True': 'get_curr', 'False': 'wait_until'},
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

            # x:303 y:66
            OperatableStateMachine.add('plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=3, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'check_running_is_done', 'retriable': 'plan'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:314 y:408
            OperatableStateMachine.add('wait_for_running',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running', 'done': 'finished', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:82 y:69
            OperatableStateMachine.add('wait_until',
                                        WaitForRunningState(wait_until_complete_rate=70, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_until', 'done': 'plan', 'collision': 'failed', 'failed': 'failed'},
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
