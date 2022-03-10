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
from task_flexbe_states.moveit_pose_plan_state import MoveItPosePlanState
from task_flexbe_states.set_DIO_state import SetDIOState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 27 2022
@author: Andy Chien
'''
class MoveToPickSM(Behavior):
	'''
	Move robot to pick object
	'''


	def __init__(self):
		super(MoveToPickSM, self).__init__()
		self.name = 'Move To Pick'

		# parameters of this behavior
		self.add_parameter('robot_name', '')
		self.add_parameter('velocity', 10)
		self.add_parameter('io_service', '/ur_hardware_interface/set_io')
		self.add_parameter('sim', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:287 y:330, x:279 y:193
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['io_pins', 'io_vals', 'pretarget_vector', 'pretarget_length', 'target_position', 'target_quaternion'])
		_state_machine.userdata.pretarget_vector = [0, 0, 1]
		_state_machine.userdata.pretarget_length = 0.05
		_state_machine.userdata.default_start_joints = []
		_state_machine.userdata.io_pins = [1]
		_state_machine.userdata.io_vals = [1]
		_state_machine.userdata.target_position = []
		_state_machine.userdata.target_quaternion = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:51 y:68
			OperatableStateMachine.add('Plan to pose',
										MoveItPosePlanState(robot_name=self.robot_name, velocity=self.velocity),
										transitions={'failed': 'failed', 'done': 'Execute trajectory'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'pretarget_vector': 'pretarget_vector', 'pretarget_length': 'pretarget_length', 'start_joints': 'default_start_joints', 'position': 'target_position', 'quaternion': 'target_quaternion', 'joint_trajectory': 'joint_trajectory'})

			# x:81 y:281
			OperatableStateMachine.add('Grasp object',
										SetDIOState(io_service=self.io_service, sim=self.sim),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pins': 'io_pins', 'vals': 'io_vals'})

			# x:51 y:183
			OperatableStateMachine.add('Execute trajectory',
										MoveItExecuteTrajectoryState(robot_name=self.robot_name),
										transitions={'done': 'Grasp object', 'failed': 'failed', 'collision': 'Plan to pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
