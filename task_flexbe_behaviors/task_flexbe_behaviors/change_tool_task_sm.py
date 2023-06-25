#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.hiwin_xeg32_gripper_api import HiwinXeg32GripperApi
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 25 2023
@author: TaiTing Tsai
'''
class ChangeToolTaskSM(Behavior):
	'''
	change endeffector tool with hiwin gripper
	'''


	def __init__(self, node):
		super(ChangeToolTaskSM, self).__init__()
		self.name = 'Change Tool Task'

		# parameters of this behavior
		self.add_parameter('namespace', '')
		self.add_parameter('group_name', 'ur_manipulator')
		self.add_parameter('joint_names', dict())

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		GetCurrentJoints.initialize_ros(node)
		HiwinXeg32GripperApi.initialize_ros(node)
		MoveItAsyncExecuteTrajectory.initialize_ros(node)
		MoveItJointsPlanState.initialize_ros(node)
		WaitForRunningState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:857 y:663, x:12 y:664
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['mode'])
		_state_machine.userdata.direction_in = 0
		_state_machine.userdata.distance = 600
		_state_machine.userdata.speed = 6000
		_state_machine.userdata.holding_stroke = 2600
		_state_machine.userdata.holding_speed = 500
		_state_machine.userdata.holding_force_little = 40
		_state_machine.userdata.flag = 1
		_state_machine.userdata.infront_sucker = [38.81, -84.56, 63.86, -69.23, -89.29, -5.65]
		_state_machine.userdata.sucker_spot = [0.6200551986694336, -1.38384706178774058, 1.0053377151489258, -1.1916464010821741, -1.5656312147723597, -0.16830856004823858]
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.planner = 'AdaptLazyPRMkDefault'
		_state_machine.userdata.exe_client = None
		_state_machine.userdata.mode = 'open'
		_state_machine.userdata.direction_out = 1
		_state_machine.userdata.distance_little = 10
		_state_machine.userdata.speed_little = 500
		_state_machine.userdata.holding_stroke_little = 10
		_state_machine.userdata.holding_speed_little = 500
		_state_machine.userdata.holding_force = 80

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('get_current_joint',
										GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
										transitions={'done': 'move_to_sucker_front', 'no_msg': 'get_current_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:443 y:354
			OperatableStateMachine.add('execute_plan2',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'wait_for_arrive', 'failed': 'get_curr_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:279 y:279
			OperatableStateMachine.add('get_curr_joint',
										GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
										transitions={'done': 'move_to_sucker_spot', 'no_msg': 'get_curr_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:606 y:608
			OperatableStateMachine.add('hold_or_release_2_check',
										HiwinXeg32GripperApi(),
										transitions={'done': 'hold_or_release_3_check', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'mode': 'mode', 'direction': 'direction_out', 'distance': 'distance_little', 'speed': 'speed_little', 'holding_stroke': 'holding_stroke_little', 'holding_speed': 'holding_speed_little', 'holding_force': 'holding_force_little', 'flag': 'flag'})

			# x:413 y:723
			OperatableStateMachine.add('hold_or_release_3_check',
										HiwinXeg32GripperApi(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'mode': 'mode', 'direction': 'direction_in', 'distance': 'distance_little', 'speed': 'speed_little', 'holding_stroke': 'holding_stroke_little', 'holding_speed': 'holding_speed_little', 'holding_force': 'holding_force_little', 'flag': 'flag'})

			# x:48 y:551
			OperatableStateMachine.add('hold_or_release_sucker',
										HiwinXeg32GripperApi(),
										transitions={'done': 'plan_back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'mode': 'mode', 'direction': 'direction_in', 'distance': 'distance', 'speed': 'speed', 'holding_stroke': 'holding_stroke', 'holding_speed': 'holding_speed', 'holding_force': 'holding_force', 'flag': 'flag'})

			# x:678 y:400
			OperatableStateMachine.add('move_back',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'wait_for_arm', 'failed': 'plan_back'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:282 y:119
			OperatableStateMachine.add('move_to_sucker_front',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'get_current_joint', 'done': 'excute_paln', 'retriable': 'move_to_sucker_front'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'infront_sucker', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:28 y:419
			OperatableStateMachine.add('move_to_sucker_spot',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'get_curr_joint', 'done': 'execute_plan2', 'retriable': 'move_to_sucker_spot'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'sucker_spot', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:253 y:610
			OperatableStateMachine.add('plan_back',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace='', planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'failed', 'done': 'move_back', 'retriable': 'plan_back'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'sucker_spot', 'target_joints': 'infront_sucker', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:809 y:511
			OperatableStateMachine.add('wait_for_arm',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=''),
										transitions={'waiting': 'wait_for_arm', 'done': 'hold_or_release_2_check', 'collision': 'plan_back', 'failed': 'hold_or_release_2_check'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:275 y:495
			OperatableStateMachine.add('wait_for_arrive',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
										transitions={'waiting': 'wait_for_arrive', 'done': 'hold_or_release_sucker', 'collision': 'get_curr_joint', 'failed': 'get_curr_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:0 y:277
			OperatableStateMachine.add('wait_for_running',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
										transitions={'waiting': 'wait_for_running', 'done': 'get_curr_joint', 'collision': 'get_current_joint', 'failed': 'get_current_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:87 y:159
			OperatableStateMachine.add('excute_paln',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'wait_for_running', 'failed': 'get_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
