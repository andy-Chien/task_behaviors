#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from task_flexbe_states.calculate_tf_state import CalculateTfState
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.get_ik_joint_state import GetIkJointState
from task_flexbe_states.get_single_armarker_state import GetSingleArmarkerState
from task_flexbe_states.get_tf_state import GetTfState
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
from task_flexbe_states.set_static_tf_state import SetStaticTfState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thur Jun 22 2023
@author: TaiTing Tsai
'''
class MobileManipulatorCalibrationDemoSM(Behavior):
	'''
	calibrate moblie manipulator tf_static
	'''


	def __init__(self, node):
		super(MobileManipulatorCalibrationDemoSM, self).__init__()
		self.name = 'Mobile Manipulator Calibration Demo'

		# parameters of this behavior
		self.add_parameter('namespace', '')
		self.add_parameter('group_name', 'ur_manipulator')
		self.add_parameter('joint_names', dict())
		self.add_parameter('random_areas', dict())
		self.add_parameter('planner_id', 'RRTConnectkConfigDefault')
		self.add_parameter('terminal_rounds', 100)
		self.add_parameter('do_evaluation', False)
		self.add_parameter('using_areas', dict())
		self.add_parameter('eval_rounds', 100)

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		CalculateTfState.initialize_ros(node)
		GetCurrentJoints.initialize_ros(node)
		GetIkJointState.initialize_ros(node)
		GetSingleArmarkerState.initialize_ros(node)
		GetTfState.initialize_ros(node)
		MoveItAsyncExecuteTrajectory.initialize_ros(node)
		MoveItJointsPlanState.initialize_ros(node)
		SetStaticTfState.initialize_ros(node)
		WaitForRunningState.initialize_ros(node)
		WaitState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:675 y:527, x:183 y:548
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity', 'planner'])
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.exe_client = None
		_state_machine.userdata.curr_area = 0
		_state_machine.userdata.running_cnt = 0
		_state_machine.userdata.goal_pos = [0.0, 0.0, 0.0]
		_state_machine.userdata.goal_rot = [0.382671, 0.923877, 0.0, 0.0]
		_state_machine.userdata.planner = "BiTRRT"
		_state_machine.userdata.transform_list = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


		with _state_machine:
			# x:588 y:154
			OperatableStateMachine.add('get_cali_armarker',
										GetSingleArmarkerState(dictionary_id_name='DICT_5X5_250', marker_size=0.053, camera_info_topic='/camera/color/camera_info', image_topic='/camera/color/image_raw'),
										transitions={'done': 'calculate_tf', 'failed': 'get_cali_armarker'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'armarker_pos': 'transform'})

			# x:399 y:566
			OperatableStateMachine.add('async_execute',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
										transitions={'done': 'wait_for_running', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:816 y:106
			OperatableStateMachine.add('calculate_tf',
										CalculateTfState(tf_right_muti_count=1, tf_left_muti_count=1, static_axis='z'),
										transitions={'done': 'update_tf', 'failed': 'calculate_tf', 'listen_tf': 'get_tf_trans'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'listen_tf': Autonomy.Off},
										remapping={'transform': 'transform', 'listened_transform': 'transform_list', 'updated_transform': 'updated_transform'})

			# x:9 y:152
			OperatableStateMachine.add('get_curr_joints',
										GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
										transitions={'done': 'get_ik_joint', 'no_msg': 'get_curr_joints'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'start_joints'})

			# x:71 y:305
			OperatableStateMachine.add('get_ik_joint',
										GetIkJointState(group_name=self.group_name, joint_names=self.joint_names, frame_id='world', namespace=self.namespace),
										transitions={'done': 'Plan', 'failed': 'get_ik_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'start_joints', 'goal_pos': 'goal_pos', 'goal_rot': 'goal_rot', 'target_joints': 'target_joints'})

			# x:585 y:304
			OperatableStateMachine.add('get_tf_trans',
										GetTfState(parent_frame=['world', 'camera_color_frame'], child_frame=['world_marker', 'base_link']),
										transitions={'done': 'calculate_tf', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'transform_list': 'transform_list'})

			# x:781 y:408
			OperatableStateMachine.add('update_tf',
										SetStaticTfState(parent_frame='world', child_frame='base_link'),
										transitions={'done': 'wait'},
										autonomy={'done': Autonomy.Off},
										remapping={'transform_mat': 'updated_transform'})

			# x:184 y:126
			OperatableStateMachine.add('wait',
										WaitState(wait_time=2),
										transitions={'done': 'get_curr_joints'},
										autonomy={'done': Autonomy.Off})

			# x:372 y:260
			OperatableStateMachine.add('wait_for_running',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
										transitions={'waiting': 'wait_for_running', 'done': 'finished', 'collision': 'get_curr_joints', 'failed': 'get_curr_joints'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:216 y:383
			OperatableStateMachine.add('Plan',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=3, namespace=self.namespace, planner=self.planner_id, time_out=0.1, attempts=1),
										transitions={'failed': 'failed', 'done': 'async_execute', 'retriable': 'Plan'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'start_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
