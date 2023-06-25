#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mm_flexbe_states.execute_trajectory_state import ExecuteTrajectoryState
from mm_flexbe_states.get_current_joints import GetCurrentJoints as mm_flexbe_states__GetCurrentJoints
from mm_flexbe_states.moveit_compute_ik import MoveItComputeIK
from mm_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState as mm_flexbe_states__MoveItJointsPlanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 14 2023
@author: Andy Chien
'''
class MoveArmToPoseSM(Behavior):
	'''
	Move arm to target pose using MoveIt and mm trajectory controller
	'''


	def __init__(self, node):
		super(MoveArmToPoseSM, self).__init__()
		self.name = 'Move Arm To Pose'

		# parameters of this behavior
		self.add_parameter('group_name', 'mobile_manipulator')
		self.add_parameter('joint_names', dict())
		self.add_parameter('namespace', '')

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		ExecuteTrajectoryState.initialize_ros(node)
		GetCurrentJoints.initialize_ros(node)
		MoveItComputeIK.initialize_ros(node)
		MoveItJointsPlanState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:600 y:610, x:252 y:439
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_pose', 'translation_list'])
		_state_machine.userdata.exe_client = None
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.target_pose = None
		_state_machine.userdata.translation_list = [0.0 ,0.0 ,0.0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


		with _state_machine:
			# x:233 y:114
			OperatableStateMachine.add('get curr',
										mm_flexbe_states__GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
										transitions={'done': 'ik', 'no_msg': 'failed'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:411 y:110
			OperatableStateMachine.add('ik',
										MoveItComputeIK(group_name=self.group_name, joint_names=self.joint_names, namespace=self.namespace),
										transitions={'done': 'plan', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_pose': 'target_pose', 'translation_list': 'translation_list', 'target_joints': 'target_joints'})

			# x:573 y:128
			OperatableStateMachine.add('plan',
										mm_flexbe_states__MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=3, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'failed', 'done': 'execute', 'retriable': 'plan'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'joint_trajectory': 'joint_trajectory', 'robot_trajectory': 'robot_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:749 y:233
			OperatableStateMachine.add('execute',
										ExecuteTrajectoryState(namespace=self.namespace),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
										remapping={'trajectory': 'joint_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
