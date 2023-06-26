#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.generate_goal_pose_state import GenerateGoalPoseState
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.get_ik_joint_state import GetIkJointState
from task_flexbe_states.get_obj_pointcloud_state import GetObjPointCloudState
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 26 2023
@author: TaiTing Tsai
'''
class TestMergeObjPointCloudSM(Behavior):
	'''
	test merge obj point cloud
	'''


	def __init__(self, node):
		super(TestMergeObjPointCloudSM, self).__init__()
		self.name = 'Test Merge Obj Point Cloud'

		# parameters of this behavior
		self.add_parameter('namespace', '')
		self.add_parameter('group_name', 'ur_manipulator')
		self.add_parameter('joint_names', dict())

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		GenerateGoalPoseState.initialize_ros(node)
		GetCurrentJoints.initialize_ros(node)
		GetIkJointState.initialize_ros(node)
		GetObjPointCloudState.initialize_ros(node)
		MoveItAsyncExecuteTrajectory.initialize_ros(node)
		MoveItJointsPlanState.initialize_ros(node)
		WaitForRunningState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:17 y:672, x:112 y:675
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.planner = 'BiTRRT'
		_state_machine.userdata.exe_client = None
		_state_machine.userdata.goal_pos = [0.613, -0.174, 0.130]
		_state_machine.userdata.goal_rot = [0.131, -0.991, 0.0, 0.0]
		_state_machine.userdata.task_list_1 = [0.0, 0.0, -0.866, 0.500]
		_state_machine.userdata.task_list_2 = [0.0, 0.0, -0.866, -0.500]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:132 y:44
			OperatableStateMachine.add('get_first_current_joint',
										GetCurrentJoints(joint_names=self.joint_names, namespace=''),
										transitions={'done': 'get_first_tool_tip_traj', 'no_msg': 'get_first_current_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:393 y:369
			OperatableStateMachine.add('execute_second_plan',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'second_waiting', 'failed': 'get_second_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:359 y:630
			OperatableStateMachine.add('execute_third_plan',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'third_waiting', 'failed': 'get_third_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

			# x:348 y:113
			OperatableStateMachine.add('first_plan',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace='', planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'get_first_current_joint', 'done': 'execute_first_plan', 'retriable': 'first_plan'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:31 y:266
			OperatableStateMachine.add('first_waiting',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=''),
										transitions={'waiting': 'first_waiting', 'done': 'merge_point_cloud', 'collision': 'get_first_current_joint', 'failed': 'get_first_current_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:356 y:30
			OperatableStateMachine.add('get_first_tool_tip_traj',
										GetIkJointState(group_name=self.group_name, joint_names=self.joint_names, frame_id='base_link', namespace=''),
										transitions={'done': 'first_plan', 'failed': 'get_first_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'goal_pos': 'goal_pos', 'goal_rot': 'goal_rot', 'target_joints': 'target_joints'})

			# x:220 y:266
			OperatableStateMachine.add('get_second_current_joint',
										GetCurrentJoints(joint_names=self.joint_names, namespace=''),
										transitions={'done': 'rotate_pose1', 'no_msg': 'get_second_current_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:560 y:225
			OperatableStateMachine.add('get_second_tool_tip_traj',
										GetIkJointState(group_name=self.group_name, joint_names=self.joint_names, frame_id='base_link', namespace=''),
										transitions={'done': 'second_plan', 'failed': 'get_second_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'goal_pos': 'update_pos', 'goal_rot': 'update_rot', 'target_joints': 'target_joints'})

			# x:226 y:468
			OperatableStateMachine.add('get_third_current_joint',
										GetCurrentJoints(joint_names=self.joint_names, namespace=''),
										transitions={'done': 'rotate_pose2', 'no_msg': 'get_third_current_joint'},
										autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
										remapping={'curr_joints': 'curr_joints'})

			# x:717 y:523
			OperatableStateMachine.add('get_third_tool_tip_traj',
										GetIkJointState(group_name=self.group_name, joint_names=self.joint_names, frame_id='base_link', namespace=''),
										transitions={'done': 'third_plan', 'failed': 'get_third_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'goal_pos': 'update_pos', 'goal_rot': 'update_rot', 'target_joints': 'target_joints'})

			# x:18 y:494
			OperatableStateMachine.add('merge_point_cloud',
										GetObjPointCloudState(depth_camera_info='/camera/depth/camera_info', depth_image_topic='/camera/depth/image_rect_raw', tip_link='tool_tip', camera_frame='camera_depth_frame'),
										transitions={'done': 'finished', 'failed': 'failed', 'second': 'get_second_current_joint', 'third': 'get_third_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'second': Autonomy.Off, 'third': Autonomy.Off},
										remapping={'obj_pointcloud': 'obj_pointcloud'})

			# x:582 y:116
			OperatableStateMachine.add('rotate_pose1',
										GenerateGoalPoseState(Rotate=True, Translate=False),
										transitions={'done': 'get_second_tool_tip_traj'},
										autonomy={'done': Autonomy.Off},
										remapping={'origin_pos': 'goal_pos', 'origin_rot': 'goal_rot', 'task_list': 'task_list_1', 'update_pos': 'update_pos', 'update_rot': 'update_rot'})

			# x:556 y:431
			OperatableStateMachine.add('rotate_pose2',
										GenerateGoalPoseState(Rotate=True, Translate=False),
										transitions={'done': 'get_third_tool_tip_traj'},
										autonomy={'done': Autonomy.Off},
										remapping={'origin_pos': 'goal_pos', 'origin_rot': 'goal_rot', 'task_list': 'task_list_2', 'update_pos': 'update_pos', 'update_rot': 'update_rot'})

			# x:715 y:356
			OperatableStateMachine.add('second_plan',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace='', planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'get_second_current_joint', 'done': 'execute_second_plan', 'retriable': 'second_plan'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:208 y:404
			OperatableStateMachine.add('second_waiting',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=''),
										transitions={'waiting': 'second_waiting', 'done': 'merge_point_cloud', 'collision': 'get_second_current_joint', 'failed': 'get_second_current_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:548 y:580
			OperatableStateMachine.add('third_plan',
										MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace='', planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
										transitions={'failed': 'get_third_current_joint', 'done': 'execute_third_plan', 'retriable': 'third_plan'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
										remapping={'start_joints': 'curr_joints', 'target_joints': 'target_joints', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

			# x:156 y:618
			OperatableStateMachine.add('third_waiting',
										WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=''),
										transitions={'waiting': 'third_waiting', 'done': 'merge_point_cloud', 'collision': 'get_third_current_joint', 'failed': 'get_third_current_joint'},
										autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exe_client': 'exe_client'})

			# x:201 y:172
			OperatableStateMachine.add('execute_first_plan',
										MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=''),
										transitions={'done': 'first_waiting', 'failed': 'get_first_current_joint'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
