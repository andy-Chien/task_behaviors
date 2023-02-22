#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.get_random_pose_in_areas_state import GetRandomPoseInAreasState
from task_flexbe_states.moveit_execute_traj_state import MoveItExecuteTrajectoryState
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
        GetRandomPoseInAreasState.initialize_ros(node)
        MoveItExecuteTrajectoryState.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.block_execute = False
        _state_machine.userdata.is_running = False
        _state_machine.userdata.velocity = 100
        _state_machine.userdata.exe_client = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:270 y:95
            OperatableStateMachine.add('get_random_joints',
                                        GetRandomPoseInAreasState(group_name=self.group_name, joint_names=self.joint_names, areas=self.random_areas, namespace=self.namespace),
                                        transitions={'done': 'Plan', 'failed': 'get_random_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'is_running': 'is_running', 'start_joints': 'start_joints', 'target_joints': 'target_joints'})

            # x:589 y:101
            OperatableStateMachine.add('execute',
                                        MoveItExecuteTrajectoryState(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'get_random_joints', 'failed': 'get_random_joints', 'collision': 'get_random_joints', 'running': 'get_random_joints'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off, 'running': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'block_execute': 'block_execute', 'is_running': 'is_running', 'exe_client': 'exe_client'})

            # x:617 y:265
            OperatableStateMachine.add('wait_execute',
                                        WaitForRunningState(namespace=self.namespace),
                                        transitions={'waiting': 'wait_execute', 'done': 'execute', 'not_running': 'execute'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'not_running': Autonomy.Off},
                                        remapping={'is_running': 'is_running', 'exe_client': 'exe_client'})

            # x:282 y:268
            OperatableStateMachine.add('Plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=1.0),
                                        transitions={'failed': 'get_random_joints', 'done': 'wait_execute'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'joint_trajectory': 'joint_trajectory'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
