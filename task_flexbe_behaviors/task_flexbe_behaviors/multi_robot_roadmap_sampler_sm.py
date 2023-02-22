#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from solomon_flexbe_behaviors.robot_roadmap_sampler_sm import RobotRoadmapSamplerSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 26 2022
@author: Andy Chien
'''
class MultiRobotRoadmapSamplerSM(Behavior):
	'''
	Roadmap sample from random start and target point in bounds
	'''


	def __init__(self):
		super(MultiRobotRoadmapSamplerSM, self).__init__()
		self.name = 'Multi Robot Roadmap Sampler'

		# parameters of this behavior
		self.add_parameter('robot_0_name', 'robot0')
		self.add_parameter('robot_1_name', 'robot1')

		# references to used behaviors
		self.add_behavior(RobotRoadmapSamplerSM, 'multi robot container/Robot Roadmap Sampler 0')
		self.add_behavior(RobotRoadmapSamplerSM, 'multi robot container/Robot Roadmap Sampler 1')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:165 y:385, x:446 y:358
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joints_home_0 = [-12, -96, -98, -75, 89, 63]
		_state_machine.userdata.joints_home_1 = [-12, -96, -98, -75, 89, 63]
		_state_machine.userdata.target_pos_min_1 = [0.32, -0.4, -0.35]
		_state_machine.userdata.target_pos_max_1 = [0.87, 0.38, -0.11]
		_state_machine.userdata.start_pos_min_1 = [0.68, -0.22, 0.04]
		_state_machine.userdata.start_pos_max_1 = [0.69, 0.22, 0.05]
		_state_machine.userdata.target_pos_min_0 = [0.3, -0.4, -0.35]
		_state_machine.userdata.target_pos_max_0 = [0.87, 0.38, -0.11]
		_state_machine.userdata.target_rot_min = [-30, 150, -180]
		_state_machine.userdata.target_rot_max = [30, 210, 180]
		_state_machine.userdata.start_pos_min_0 = [0.49, -0.22, 0.03]
		_state_machine.userdata.start_pos_max_0 = [0.5, 0.22, 0.04]
		_state_machine.userdata.start_rot_min = [-5, 175, -180]
		_state_machine.userdata.start_rot_max = [5, 185, 180]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:302 y:340, x:315 y:230, x:488 y:348, x:484 y:308
		_sm_multi_robot_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['joints_home_0', 'joints_home_1', 'target_pos_min_0', 'target_pos_max_0', 'target_rot_min_0', 'target_rot_max_0', 'start_pos_min_0', 'start_pos_max_0', 'start_rot_min_0', 'start_rot_max_0', 'target_pos_min_1', 'target_pos_max_1', 'target_rot_min_1', 'target_rot_max_1', 'start_pos_min_1', 'start_pos_max_1', 'start_rot_min_1', 'start_rot_max_1'], conditions=[
										('failed', [('Robot Roadmap Sampler 0', 'failed'), ('Robot Roadmap Sampler 1', 'failed')]),
										('finished', [('Robot Roadmap Sampler 0', 'finished'), ('Robot Roadmap Sampler 1', 'finished')])
										])

		with _sm_multi_robot_container_0:
			# x:86 y:157
			OperatableStateMachine.add('Robot Roadmap Sampler 0',
										self.use_behavior(RobotRoadmapSamplerSM, 'multi robot container/Robot Roadmap Sampler 0',
											parameters={'robot_name': self.robot_0_name}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joints_home': 'joints_home_0', 'target_pos_min': 'target_pos_min_0', 'target_pos_max': 'target_pos_max_0', 'target_rot_min': 'target_rot_min_0', 'target_rot_max': 'target_rot_max_0', 'start_pos_min': 'start_pos_min_0', 'start_pos_max': 'start_pos_max_0', 'start_rot_min': 'start_rot_min_0', 'start_rot_max': 'start_rot_max_0'})

			# x:401 y:159
			OperatableStateMachine.add('Robot Roadmap Sampler 1',
										self.use_behavior(RobotRoadmapSamplerSM, 'multi robot container/Robot Roadmap Sampler 1',
											parameters={'robot_name': self.robot_1_name}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joints_home': 'joints_home_1', 'target_pos_min': 'target_pos_min_1', 'target_pos_max': 'target_pos_max_1', 'target_rot_min': 'target_rot_min_1', 'target_rot_max': 'target_rot_max_1', 'start_pos_min': 'start_pos_min_1', 'start_pos_max': 'start_pos_max_1', 'start_rot_min': 'start_rot_min_1', 'start_rot_max': 'start_rot_max_1'})



		with _state_machine:
			# x:208 y:151
			OperatableStateMachine.add('multi robot container',
										_sm_multi_robot_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joints_home_0': 'joints_home_0', 'joints_home_1': 'joints_home_1', 'target_pos_min_0': 'target_pos_min_0', 'target_pos_max_0': 'target_pos_max_0', 'target_rot_min_0': 'target_rot_min', 'target_rot_max_0': 'target_rot_max', 'start_pos_min_0': 'start_pos_min_0', 'start_pos_max_0': 'start_pos_max_0', 'start_rot_min_0': 'start_rot_min', 'start_rot_max_0': 'start_rot_max', 'target_pos_min_1': 'target_pos_min_1', 'target_pos_max_1': 'target_pos_max_1', 'target_rot_min_1': 'target_rot_min', 'target_rot_max_1': 'target_rot_max', 'start_pos_min_1': 'start_pos_min_1', 'start_pos_max_1': 'start_pos_max_1', 'start_rot_min_1': 'start_rot_min', 'start_rot_max_1': 'start_rot_max'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
