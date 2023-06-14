#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 14 2023
@author: TaiTing Tsai
'''
class TestGQCNNSM(Behavior):
	'''
	test gqcnn
	'''


	def __init__(self, node):
		super(TestGQCNNSM, self).__init__()
		self.name = 'Test GQCNN'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		GQCNNGraspPlanState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:154 y:158
			OperatableStateMachine.add('gqcnn',
										GQCNNGraspPlanState(grasp_service='/gqcnn/grasp_planner', depth_camera_info='/depth_to_rgb/camera_info', color_image_topic='/rgb/image_raw', depth_image_topic='/depth_to_rgb/image_raw'),
										transitions={'done': 'finished', 'failed': 'failed', 'retry': 'gqcnn'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
										remapping={'grasp_position': 'grasp_position', 'grasp_quaternion': 'grasp_quaternion'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
