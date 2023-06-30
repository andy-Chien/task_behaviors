#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mm_flexbe_states.convert_path_to_trajectory import ConvertPathToTrajectory
from mm_flexbe_states.execute_trajectory_state import ExecuteTrajectoryState
from mm_flexbe_states.get_current_joints import GetCurrentJoints
from mm_flexbe_states.merge_base_arm_trajectory import MergeBaseArmTrajectory
from mm_flexbe_states.moveit_compute_ik import MoveItComputeIK
from mm_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from mm_flexbe_states.nav2_path_plan_state import Nav2PathPlanState
from mm_flexbe_states.pose_to_base_and_arm import PoseToBaseAndArm
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 21 2023
@author: Andy Chien
'''
class MoveMMToPoseSM(Behavior):
    '''
    Move mobile manipulator to a pose
    '''


    def __init__(self, node):
        super(MoveMMToPoseSM, self).__init__()
        self.name = 'Move MM To Pose'

        # parameters of this behavior
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'mobile_manipulator')
        self.add_parameter('joint_names', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ConvertPathToTrajectory.initialize_ros(node)
        ExecuteTrajectoryState.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        MergeBaseArmTrajectory.initialize_ros(node)
        MoveItComputeIK.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        Nav2PathPlanState.initialize_ros(node)
        PoseToBaseAndArm.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:96 y:380, x:381 y:183
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity', 'mm_pose'])
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.mm_pose = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:30 y:128
            OperatableStateMachine.add('get_target_pose',
                                        PoseToBaseAndArm(),
                                        transitions={'done': 'get_base_path'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'mm_pose': 'mm_pose', 'base_pose': 'base_pose', 'arm_pose': 'arm_pose'})

            # x:73 y:271
            OperatableStateMachine.add('execute',
                                        ExecuteTrajectoryState(namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'trajectory': 'mm_trajectory'})

            # x:473 y:43
            OperatableStateMachine.add('get_arm_curr',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'arm_ik', 'no_msg': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'start_joints'})

            # x:556 y:264
            OperatableStateMachine.add('get_arm_traj',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=3, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'merge_traj', 'retriable': 'get_arm_traj'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'joint_trajectory': 'arm_trajectory', 'robot_trajectory': 'robot_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:68 y:41
            OperatableStateMachine.add('get_base_path',
                                        Nav2PathPlanState(namespace=self.namespace, planner_id='GridBased'),
                                        transitions={'failed': 'failed', 'done': 'get_base_traj'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'target_pose': 'base_pose', 'base_path': 'base_path'})

            # x:252 y:42
            OperatableStateMachine.add('get_base_traj',
                                        ConvertPathToTrajectory(namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'get_arm_curr'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'base_path': 'base_path', 'base_trajectory': 'base_trajectory'})

            # x:309 y:322
            OperatableStateMachine.add('merge_traj',
                                        MergeBaseArmTrajectory(namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'execute'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'base_traj': 'base_trajectory', 'arm_traj': 'arm_trajectory', 'mm_trajectory': 'mm_trajectory'})

            # x:623 y:41
            OperatableStateMachine.add('arm_ik',
                                        MoveItComputeIK(group_name=self.group_name, joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'get_arm_traj', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'start_joints': 'start_joints', 'target_pose': 'arm_pose', 'target_joints': 'target_joints'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
