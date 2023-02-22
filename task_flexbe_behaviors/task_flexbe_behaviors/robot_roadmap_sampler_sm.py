#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from solomon_flexbe_behaviors.scenemanagerplan_sm import SceneManagerPlanSM
from solomon_flexbe_states.get_robot_mesh_path_state import GetRobotMeshPath
from solomon_flexbe_states.init_robot_mesh_state import InitRobotMeshState
from solomon_flexbe_states.set_random_pose_state import SetRandomPoseState
from solomon_flexbe_states.set_robot_init_mesh_state import SetRobotInitMeshState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 19 2022
@author: Andy Chien
'''
class RobotRoadmapSamplerSM(Behavior):
	'''
	Roadmap sample for pick and place task
	'''


	def __init__(self):
		super(RobotRoadmapSamplerSM, self).__init__()
		self.name = 'Robot Roadmap Sampler'

		# parameters of this behavior
		self.add_parameter('robot_name', 'robot0')
		self.add_parameter('planning_timeout', 0.3)

		# references to used behaviors
		self.add_behavior(SceneManagerPlanSM, 'Scene_Manager_Plan')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:390 y:743, x:483 y:92
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joints_home', 'target_pos_min', 'target_pos_max', 'target_rot_min', 'target_rot_max', 'start_pos_min', 'start_pos_max', 'start_rot_min', 'start_rot_max'])
		_state_machine.userdata.joints_home = [-12.42,-96.67,-98.03,-75.69,89.68,63.59]
		_state_machine.userdata.target_pos_min = [0.3, -0.4, -0.35]
		_state_machine.userdata.target_pos_max = [0.87, 0.38, -0.11]
		_state_machine.userdata.target_rot_min = [-30, 150, -180]
		_state_machine.userdata.target_rot_max = [30, 210, 180]
		_state_machine.userdata.start_pos_min = [0.49, -0.22, 0.03]
		_state_machine.userdata.start_pos_max = [0.5, 0.22, 0.04]
		_state_machine.userdata.start_rot_min = [-5, 175, -180]
		_state_machine.userdata.start_rot_max = [5, 185, 180]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:174 y:80
			OperatableStateMachine.add('get_robot_mesh_path',
										GetRobotMeshPath(ns=self.robot_name, param_name=self.robot_name+'/robot_description'),
										transitions={'done': 'init_robot_mesh', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'paths': 'robot_mesh_filepath', 'tool_paths': 'tool_mesh_filepath'})

			# x:180 y:202
			OperatableStateMachine.add('init_robot_mesh',
										InitRobotMeshState(namespace=self.robot_name, mesh_scale=[1.1,1.1,1.1]),
										transitions={'done': 'set_mesh_init', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints': 'joints_home', 'robot_mesh_filepath': 'robot_mesh_filepath', 'tool_mesh_filepath': 'tool_mesh_filepath'})

			# x:178 y:357
			OperatableStateMachine.add('set_mesh_init',
										SetRobotInitMeshState(robot_name=self.robot_name, robot_id_key='robot_co', tool_id_key='tool_co'),
										transitions={'done': 'set_random_start_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:435 y:263
			OperatableStateMachine.add('set_random_start_pose',
										SetRandomPoseState(namespace=self.robot_name),
										transitions={'done': 'set_random_target_pose', 'failed': 'set_random_start_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pos_min': 'start_pos_min', 'pos_max': 'start_pos_max', 'rot_min': 'start_rot_min', 'rot_max': 'start_rot_max', 'init_joints': 'joints_home', 'sampled_joints': 'start_joints'})

			# x:434 y:359
			OperatableStateMachine.add('set_random_target_pose',
										SetRandomPoseState(namespace=self.robot_name),
										transitions={'done': 'Scene_Manager_Plan', 'failed': 'set_random_start_pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pos_min': 'target_pos_min', 'pos_max': 'target_pos_max', 'rot_min': 'target_rot_min', 'rot_max': 'target_rot_max', 'init_joints': 'start_joints', 'sampled_joints': 'target_joints'})

			# x:672 y:76
			OperatableStateMachine.add('Scene_Manager_Plan',
										self.use_behavior(SceneManagerPlanSM, 'Scene_Manager_Plan',
											parameters={'robot_name': self.robot_name, 'planning_timeout': self.planning_timeout}),
										transitions={'finished': 'set_random_start_pose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'start_joints': 'start_joints', 'target': 'target_joints', 'trajectory': 'trajectory', 'collision_trajectory': 'collision_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
