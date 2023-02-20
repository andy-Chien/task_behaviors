#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.moveit_execute_traj_state import MoveItExecuteTrajectoryState
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.set_random_pose_state import SetRandomPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 20 2023
@author: Andy Chien
'''
class SingleArmRandomSampledPlanningSM(Behavior):
    '''
    single robot arm planning for the start and targert pose sampled from two area
    '''


    def __init__(self, node):
        super(SingleArmRandomSampledPlanningSM, self).__init__()
        self.name = 'Single Arm Random Sampled Planning'

        # parameters of this behavior
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('test_file', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        MoveItExecuteTrajectoryState.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        SetRandomPoseState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.start_pos_min = [0.49, -0.22, -0.03]
        _state_machine.userdata.start_pos_max = [0.5, 0.22, 0.04]
        _state_machine.userdata.start_rot_min = [-5, 175, -180]
        _state_machine.userdata.start_rot_max = [5, 185, 180]
        _state_machine.userdata.target_pos_min = [0.3, -0.4, -0.35]
        _state_machine.userdata.target_pos_max = [0.87, 0.38, -0.11]
        _state_machine.userdata.target_rot_min = [-5, 175, -180]
        _state_machine.userdata.target_rot_max = [5, 185, 180]
        _state_machine.userdata.home_joints = [0.0, -90, -90, -90, 0, 0]
        _state_machine.userdata.block_execute = False

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:36 y:80
            OperatableStateMachine.add('set_random_start_pose',
                                        SetRandomPoseState(group_name=self.group_name, joint_names=joint_names, namespace=self.namespace),
                                        transitions={'done': 'set_random_target_pose', 'failed': 'set_random_start_pose'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pos_min': 'start_pos_min', 'pos_max': 'start_pos_max', 'rot_min': 'start_rot_min', 'rot_max': 'start_rot_max', 'init_joints': 'home_joints', 'sampled_joints': 'start_joints'})

            # x:516 y:189
            OperatableStateMachine.add('execute',
                                        MoveItExecuteTrajectoryState(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'set_random_start_pose', 'failed': 'set_random_start_pose', 'collision': 'set_random_start_pose', 'running': 'set_random_start_pose'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off, 'running': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'block_execute': 'block_execute'})

            # x:30 y:212
            OperatableStateMachine.add('set_random_target_pose',
                                        SetRandomPoseState(group_name=self.group_name, joint_names=joint_names, namespace=self.namespace),
                                        transitions={'done': 'Plan', 'failed': 'set_random_start_pose'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pos_min': 'target_pos_min', 'pos_max': 'target_pos_max', 'rot_min': 'target_rot_min', 'rot_max': 'target_rot_max', 'init_joints': 'home_joints', 'sampled_joints': 'target_joints'})

            # x:303 y:231
            OperatableStateMachine.add('Plan',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=joint_names, namespace=self.namespace, planner='RRTConnectkConfigDefault', velocity=0.1, time_out=1.0),
                                        transitions={'failed': 'set_random_start_pose', 'done': 'execute'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'joint_trajectory': 'joint_trajectory'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
