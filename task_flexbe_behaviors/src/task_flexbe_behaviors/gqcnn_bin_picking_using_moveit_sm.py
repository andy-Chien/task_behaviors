#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_behaviors.go_to_initial_pose_sm import GoToInitialPoseSM
from task_flexbe_behaviors.grasp_plan_sm import GraspPlanSM
from task_flexbe_behaviors.move_to_pick_sm import MoveToPickSM
from task_flexbe_behaviors.move_to_place_sm import MoveToPlaceSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 27 2022
@author: Andy Chien
'''
class GQCNNBinPickingUsingMoveItSM(Behavior):
	'''
	Using MoveIt to plan and execute robot to run bin picking task.
	'''


	def __init__(self):
		super(GQCNNBinPickingUsingMoveItSM, self).__init__()
		self.name = 'GQCNN Bin Picking Using MoveIt'

		# parameters of this behavior
		self.add_parameter('robot_name', 'ur5')
		self.add_parameter('velocity', 10)
		self.add_parameter('sim', True)

		# references to used behaviors
		self.add_behavior(GoToInitialPoseSM, 'Go To Initial Pose')
		self.add_behavior(GraspPlanSM, 'Grasp Plan')
		self.add_behavior(MoveToPickSM, 'Move To Pick')
		self.add_behavior(MoveToPlaceSM, 'Move To Place')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:663 y:261, x:262 y:61
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:43 y:56
			OperatableStateMachine.add('Go To Initial Pose',
										self.use_behavior(GoToInitialPoseSM, 'Go To Initial Pose',
											parameters={'robot_name': self.robot_name, 'velocity': self.velocity}),
										transitions={'finished': 'Grasp Plan', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:40 y:239
			OperatableStateMachine.add('Grasp Plan',
										self.use_behavior(GraspPlanSM, 'Grasp Plan'),
										transitions={'finished': 'Move To Pick', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:399 y:67
			OperatableStateMachine.add('Move To Pick',
										self.use_behavior(MoveToPickSM, 'Move To Pick',
											parameters={'robot_name': self.robot_name, 'velocity': self.velocity, 'sim': self.sim}),
										transitions={'finished': 'Move To Place', 'failed': 'Grasp Plan'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:401 y:231
			OperatableStateMachine.add('Move To Place',
										self.use_behavior(MoveToPlaceSM, 'Move To Place',
											parameters={'robot_name': self.robot_name, 'sim': self.sim}),
										transitions={'finished': 'Grasp Plan', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
