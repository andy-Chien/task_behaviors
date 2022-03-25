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
from task_flexbe_states.set_place_pose_random_state import SetPlacePoseRandomState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 27 2022
@author: Andy Chien
'''
class MoveToPlaceSM(Behavior):
	'''
	Move robot to place object
	'''


	def __init__(self):
		super(MoveToPlaceSM, self).__init__()
		self.name = 'Move To Place'

		# parameters of this behavior
		self.add_parameter('robot_name', '')
		self.add_parameter('io_service', '/ur_hardware_interface/set_io')
		self.add_parameter('sim', True)
		self.add_parameter('velocity', 10)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:600 y:267, x:309 y:327
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['vacuum_io_pins', 'vacuum_io_vals', 'place_pos_max', 'place_pos_min', 'place_quat', 'prestart_vector', 'prestart_length'])
		_state_machine.userdata.place_pos_max = [0, 0, 0]
		_state_machine.userdata.place_pos_min = [0, 0, 0]
		_state_machine.userdata.place_quat = [1, 0, 0, 0]
		_state_machine.userdata.default_start_joints = []
		_state_machine.userdata.vacuum_io_pins = [1]
		_state_machine.userdata.vacuum_io_vals = [0]
		_state_machine.userdata.pretarget_vector = [0, 0, 0]
		_state_machine.userdata.pretarget_length = 0
		_state_machine.userdata.prestart_vector = [0, 0, 0]
		_state_machine.userdata.prestart_length = 0
		_state_machine.userdata.block_execute = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:52
			OperatableStateMachine.add('Set place pose',
										SetPlacePoseRandomState(),
										transitions={'done': 'Plan to pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'place_position_max': 'place_pos_max', 'place_position_min': 'place_pos_min', 'place_position': 'place_position'})

			# x:531 y:138
			OperatableStateMachine.add('Place the object',
										SetDIOState(io_service=self.io_service, sim=self.sim),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

			# x:59 y:153
			OperatableStateMachine.add('Plan to pose',
										MoveItPosePlanState(robot_name=self.robot_name, velocity=self.velocity),
										transitions={'failed': 'failed', 'done': 'Execute trajectory'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'prestart_length': 'prestart_length', 'prestart_vector': 'prestart_vector', 'pretarget_length': 'pretarget_length', 'pretarget_vector': 'pretarget_vector', 'start_joints': 'default_start_joints', 'position': 'place_position', 'quaternion': 'place_quat', 'joint_trajectory': 'joint_trajectory'})

			# x:298 y:139
			OperatableStateMachine.add('Execute trajectory',
										MoveItExecuteTrajectoryState(robot_name=self.robot_name),
										transitions={'done': 'Place the object', 'failed': 'failed', 'collision': 'Plan to pose', 'running': 'Execute trajectory'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off, 'running': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'block_execute': 'block_execute'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
