#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.get_DIO_state import GetDIOState as task_flexbe_states__GetDIOState
from task_flexbe_states.grasped_and_stop import GraspedAndStop
from task_flexbe_states.moveit_execute_traj_state import MoveItExecuteTrajectoryState as task_flexbe_states__MoveItExecuteTrajectoryState
from task_flexbe_states.moveit_pose_plan_state import MoveItPosePlanState as task_flexbe_states__MoveItPosePlanState
from task_flexbe_states.set_DIO_state import SetDIOState as task_flexbe_states__SetDIOState
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


	def __init__(self, node):
		super(MoveToPickSM, self).__init__()
		self.name = 'Move To Pick'

		# parameters of this behavior
		self.add_parameter('robot_name', '')
		self.add_parameter('velocity', 10)
		self.add_parameter('io_service', '/ur_hardware_interface/set_io')
		self.add_parameter('sim', True)
		self.add_parameter('io_topic', '/ur_hardware_interface/io_states')

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		GetDIOState.initialize_ros(node)
		GraspedAndStop.initialize_ros(node)
		MoveItExecuteTrajectoryState.initialize_ros(node)
		MoveItPosePlanState.initialize_ros(node)
		SetDIOState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:321 y:374, x:658 y:157
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['vacuum_io_pins', 'vacuum_io_vals', 'pretarget_vector', 'pretarget_length', 'target_position', 'target_quaternion', 'pressure_io_pins'])
		_state_machine.userdata.pretarget_vector = [0, 0, 1]
		_state_machine.userdata.pretarget_length = 0.05
		_state_machine.userdata.default_start_joints = []
		_state_machine.userdata.vacuum_io_pins = [1]
		_state_machine.userdata.vacuum_io_vals = [1]
		_state_machine.userdata.target_position = []
		_state_machine.userdata.target_quaternion = []
		_state_machine.userdata.prestart_vector = [0, 0, 0]
		_state_machine.userdata.prestart_length = 0
		_state_machine.userdata.block_execute = False
		_state_machine.userdata.pressure_io_pins = [2]
		_state_machine.userdata.exe_client = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:236 y:61
			OperatableStateMachine.add('Plan to pose',
										task_flexbe_states__MoveItPosePlanState(robot_name=self.robot_name, velocity=self.velocity),
										transitions={'failed': 'failed', 'done': 'Grasp object'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'prestart_length': 'prestart_length', 'prestart_vector': 'prestart_vector', 'pretarget_length': 'pretarget_length', 'pretarget_vector': 'pretarget_vector', 'start_joints': 'default_start_joints', 'position': 'target_position', 'quaternion': 'target_quaternion', 'joint_trajectory': 'joint_trajectory'})

			# x:27 y:189
			OperatableStateMachine.add('Get pressure state',
										task_flexbe_states__GetDIOState(io_topic=self.io_topic, sim=self.sim),
										transitions={'done': 'Stop when grasped'},
										autonomy={'done': Autonomy.Off},
										remapping={'pins': 'pressure_io_pins', 'vals': 'vals'})

			# x:338 y:125
			OperatableStateMachine.add('Grasp object',
										task_flexbe_states__SetDIOState(io_service=self.io_service, sim=self.sim),
										transitions={'done': 'Execute trajectory', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

			# x:25 y:291
			OperatableStateMachine.add('Stop when grasped',
										GraspedAndStop(robot_name=self.robot_name, sim=self.sim),
										transitions={'done': 'Execute trajectory'},
										autonomy={'done': Autonomy.Off},
										remapping={'vals': 'vals'})

			# x:207 y:196
			OperatableStateMachine.add('Execute trajectory',
										task_flexbe_states__MoveItExecuteTrajectoryState(group_name='manipulator', namespace=''),
										transitions={'done': 'finished', 'failed': 'failed', 'collision': 'Plan to pose', 'running': 'Get pressure state'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'collision': Autonomy.Off, 'running': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'block_execute': 'block_execute', 'exe_client': 'exe_client', 'is_running': 'is_running'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
