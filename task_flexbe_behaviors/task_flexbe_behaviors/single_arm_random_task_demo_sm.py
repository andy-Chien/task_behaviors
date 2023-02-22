#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.data_copy_state import DataCopyState
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.get_random_pose_in_areas_state import GetRandomPoseInAreasState
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
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
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('random_areas', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DataCopyState.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        GetRandomPoseInAreasState.initialize_ros(node)
        MoveItAsyncExecuteTrajectory.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:65 y:538, x:132 y:522
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.velocity = 100
        _state_machine.userdata.exe_client = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:98 y:82
            OperatableStateMachine.add('get_curr_joints',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'get_random_joints', 'no_msg': 'get_curr_joints'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'start_joints'})

            # x:598 y:68
            OperatableStateMachine.add('async_execute',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'set_last_target_to_new_start', 'failed': 'get_curr_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:96 y:236
            OperatableStateMachine.add('get_random_joints',
                                        GetRandomPoseInAreasState(group_name=self.group_name, joint_names=self.joint_names, areas=self.random_areas, namespace=self.namespace),
                                        transitions={'done': 'Plan', 'failed': 'get_random_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints'})

            # x:366 y:122
            OperatableStateMachine.add('set_last_target_to_new_start',
                                        DataCopyState(),
                                        transitions={'done': 'get_random_joints'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data_in': 'target_joints', 'data_out': 'start_joints'})

            # x:569 y:358
            OperatableStateMachine.add('wait_for_running',
                                        WaitForRunningState(namespace=''),
                                        transitions={'waiting': 'wait_for_running', 'done': 'async_execute', 'collision': 'get_curr_joints', 'failed': 'get_curr_joints'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:103 y:361
            OperatableStateMachine.add('Plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=1.0),
                                        transitions={'failed': 'get_random_joints', 'done': 'wait_for_running'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'joint_trajectory': 'joint_trajectory'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
