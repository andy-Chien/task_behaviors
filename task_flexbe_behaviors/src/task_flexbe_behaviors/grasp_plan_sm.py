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


	def __init__(self):
		super(GraspPlanSM, self).__init__()
		self.name = 'Grasp Plan'

		# parameters of this behavior
		self.add_parameter('grasp_service', '/gqcnn/grasp_planner')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:463, x:378 y:140
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['trans_position', 'trans_quaternion'], output_keys=['target_position', 'target_quaternion'])
		_state_machine.userdata.trans_position = [0, 0, 0]
		_state_machine.userdata.trans_quaternion = [1, 0, 0, 0]
		_state_machine.userdata.target_position = []
		_state_machine.userdata.target_quaternion = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:45 y:68
			OperatableStateMachine.add('GQCNN client',
										GQCNNGraspPlanState(grasp_service=self.grasp_service),
										transitions={'done': 'Coordinate transform', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'grasp_posision': 'grasp_position', 'grasp_quaternion': 'grasp_quaternion'})

			# x:46 y:206
			OperatableStateMachine.add('Coordinate transform',
										CoordinateTransformState(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'trans_position': 'trans_position', 'trans_quaternion': 'trans_quaternion', 'source_position': 'grasp_position', 'source_quaternion': 'grasp_quaternion', 'target_position': 'target_position', 'target_quaternion': 'target_quaternion'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
