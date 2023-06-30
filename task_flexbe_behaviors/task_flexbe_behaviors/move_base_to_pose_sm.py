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
from mm_flexbe_states.nav2_path_plan_state import Nav2PathPlanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 13 2023
@author: Andy Chien
'''
class MoveBaseToPoseSM(Behavior):
    '''
    Move base to target pose using nav2 and mm trajectory controller.
    '''


    def __init__(self, node):
        super(MoveBaseToPoseSM, self).__init__()
        self.name = 'Move Base To Pose'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ConvertPathToTrajectory.initialize_ros(node)
        ExecuteTrajectoryState.initialize_ros(node)
        Nav2PathPlanState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:421 y:175, x:257 y:192
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose'])
        _state_machine.userdata.target_pose = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:45 y:64
            OperatableStateMachine.add('get_base_path',
                                        Nav2PathPlanState(namespace='', planner_id='GridBased'),
                                        transitions={'failed': 'failed', 'done': 'trajectory_from_path'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'target_pose': 'target_pose', 'base_path': 'base_path'})

            # x:203 y:67
            OperatableStateMachine.add('trajectory_from_path',
                                        ConvertPathToTrajectory(namespace=''),
                                        transitions={'failed': 'failed', 'done': 'execute_trajectory'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'base_path': 'base_path', 'base_trajectory': 'base_trajectory'})

            # x:377 y:67
            OperatableStateMachine.add('execute_trajectory',
                                        ExecuteTrajectoryState(namespace=''),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'trajectory': 'base_trajectory'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
