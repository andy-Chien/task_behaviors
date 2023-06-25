#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mm_flexbe_behaviors.move_arm_to_pose_sm import MoveArmToPoseSM
from task_flexbe_states.get_current_joints import GetCurrentJoints as task_flexbe_states__GetCurrentJoints
from task_flexbe_states.get_ik_joint_state import GetIkJointState
from task_flexbe_states.get_mask_image_state import GetMaskImageState
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory as task_flexbe_states__MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState as task_flexbe_states__MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState as task_flexbe_states__WaitForRunningState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jun 25 2023
@author: TaiTing Tsai
'''
class MutilArmGqcnnTestSM(Behavior):
	'''
	The demo of mutil arm doing Gqcnn bin picking
	'''


	def __init__(self, node):
		super(MutilArmGqcnnTestSM, self).__init__()
		self.name = 'Mutil Arm Gqcnn Test'

		# parameters of this behavior

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		GQCNNGraspPlanState.initialize_ros(node)
		task_flexbe_states__GetCurrentJoints.initialize_ros(node)
		GetIkJointState.initialize_ros(node)
		GetMaskImageState.initialize_ros(node)
		task_flexbe_states__MoveItAsyncExecuteTrajectory.initialize_ros(node)
		task_flexbe_states__MoveItJointsPlanState.initialize_ros(node)
		task_flexbe_states__WaitForRunningState.initialize_ros(node)
		self.add_behavior(MoveArmToPoseSM, 'Grasp Up', node)
		self.add_behavior(MoveArmToPoseSM, 'Move to object_up', node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:24 y:632, x:430 y:117
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goal_pos = [0.0, 0.0, 0.0]
		_state_machine.userdata.goal_rot = [0.0, 0.0, 0.0, 1.0]
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.planner = 'BiTRRT'
		_state_machine.userdata.exe_client = ''
		_state_machine.userdata.pins = 0
		_state_machine.userdata.vals = 0
		_state_machine.userdata.vacuum_io_pins = 0
		_state_machine.userdata.down_to_grasp_translation = [0.0, 0.0, 0.0]
		_state_machine.userdata.grasp_up_translation = [0.0, 0.0, 0.1]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:180 y:52
			OperatableStateMachine.add('get_mask_image',
										GetMaskImageState(mask_service='/image_masking', namespace='id1'),
										transitions={'done': 'get_grasping_pose', 'failed': 'failed', 'retry': 'get_mask_image'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
										remapping={'mask_imgmsg': 'mask_imgmsg'})

			# x:465 y:178
			OperatableStateMachine.add('Move to object_up',
										self.use_behavior(MoveArmToPoseSM, 'Move to object_up'),
										transitions={'finished': 'get_current_joint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_pose': 'grasp_pos', 'translation_list': 'down_to_grasp_translation'})

			# x:446 y:415
			OperatableStateMachine.add('Plan',
										task_flexbe_states__MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace='', planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'Plan', 'done': 'execute_plan', 'retriable': 'get_current_joint'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:176 y:306
			OperatableStateMachine.add('execute_plan',
										task_flexbe_states__MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'waiting', 'failed': 'execute_plan'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:306 y:234
			OperatableStateMachine.add('get_current_joint',
										task_flexbe_states__GetCurrentJoints(joint_names=self.joint_name, namespace=''),
										transitions={'done': 'get_ik_joints', 'no_msg': 'get_current_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:605 y:31
			OperatableStateMachine.add('get_grasping_pose',
										GQCNNGraspPlanState(grasp_service='/gqcnn/grasp_planner', depth_camera_info='/depth_to_rgb/camera_info', color_image_topic='/rgb/image_raw', depth_image_topic='/depth_to_rgb/image_raw'),
										transitions={'done': 'Move to object_up', 'failed': 'failed', 'retry': 'get_grasping_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
										remapping={'mask_imgmsg': 'mask_imgmsg', 'grasp_pos': 'grasp_pos'})

			# x:634 y:289
			OperatableStateMachine.add('get_ik_joints',
										GetIkJointState(group_name=self.group_name, joint_names=self.joint_names, frame_id='', namespace=''),
										transitions={'done': 'Plan', 'failed': 'get_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'goal_pos': 'goal_pos', 'goal_rot': 'goal_rot', 'target_joints': 'target_joints'})

			# x:215 y:485
			OperatableStateMachine.add('waiting',
										task_flexbe_states__WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=''),
										transitions={'waiting': 'waiting', 'done': 'Grasp Up', 'collision': 'get_current_joint', 'failed': 'get_current_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:19 y:195
			OperatableStateMachine.add('Grasp Up',
										self.use_behavior(MoveArmToPoseSM, 'Grasp Up'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'target_pose': 'grasp_pos', 'translation_list': 'grasp_up_translation'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
