#!/usr/bin/env python3
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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Feb 26 2022
@author: Andy Chien
'''
class GoToInitialPoseSM(Behavior):
	'''
	Move robot to the initial pose
	'''


	def __init__(self):
		super(GoToInitialPoseSM, self).__init__()
		self.name = 'Go To Initial Pose'

		# parameters of this behavior
		self.add_parameter('robot_name', '')
		self.add_parameter('velocity', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:305 y:164
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['init_joints'])
		_state_machine.userdata.init_joints = [0, 0, 0, 0, 0, 0]
		_state_machine.userdata.default_start_joints = []
		_state_machine.userdata.block_execute = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:67 y:65
			OperatableStateMachine.add('Plan to joints',
										MoveItJointsPlanState(robot_name=self.robot_name, velocity=self.velocity),
										transitions={'failed': 'failed', 'done': 'Execute trajectory'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'start_joints': 'default_start_joints', 'target_joints': 'init_joints', 'joint_trajectory': 'joint_trajectory'})

			# x:47 y:181
			OperatableStateMachine.add('Execute trajectory',
										MoveItExecuteTrajectoryState(robot_name=self.robot_name),
										transitions={'done': 'finished', 'failed': 'failed', 'collision': 'Plan to joints', 'running': 'Execute trajectory'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off, 'running': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'block_execute': 'block_execute'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
