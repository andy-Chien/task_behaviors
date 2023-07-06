#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.coordinate_transform_state import CoordinateTransformState
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Feb 26 2022
@author: Andy Chien
'''
class GraspPlanSM(Behavior):
    '''
    Plan a pose for grasping
    '''


    def __init__(self, node):
        super(GraspPlanSM, self).__init__()
        self.name = 'Grasp Plan'

        # parameters of this behavior
        self.add_parameter('grasp_service', '/gqcnn/grasp_planner')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        CoordinateTransformState.initialize_ros(node)
        GQCNNGraspPlanState.initialize_ros(node)
        Logger.initialize(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:30 y:463, x:378 y:140, x:334 y:280
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'no_object'], input_keys=['trans_position', 'trans_quaternion', 'obj_quat_tool'], output_keys=['target_position', 'target_quaternion'])
        _state_machine.userdata.trans_position = [0, 0, 0]
        _state_machine.userdata.trans_quaternion = [1, 0, 0, 0]
        _state_machine.userdata.target_position = []
        _state_machine.userdata.target_quaternion = []
        _state_machine.userdata.obj_quat_tool = [1, 0, 0, 0]

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:45 y:68
            OperatableStateMachine.add('GQCNN client',
                                        GQCNNGraspPlanState(grasp_service=self.grasp_service),
                                        transitions={'done': 'Coordinate transform', 'failed': 'failed', 'finish': 'no_object', 'retry': 'GQCNN client'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'finish': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'grasp_position': 'grasp_position', 'grasp_quaternion': 'grasp_quaternion'})

            # x:46 y:206
            OperatableStateMachine.add('Coordinate transform',
                                        CoordinateTransformState(),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'trans_position': 'trans_position', 'trans_quaternion': 'trans_quaternion', 'source_position': 'grasp_position', 'source_quaternion': 'grasp_quaternion', 'obj_quat_tool': 'obj_quat_tool', 'target_position': 'target_position', 'target_quaternion': 'target_quaternion'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
