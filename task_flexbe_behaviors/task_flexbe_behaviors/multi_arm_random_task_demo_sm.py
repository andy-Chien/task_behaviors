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
from task_flexbe_behaviors.single_arm_random_task_demo_sm import SingleArmRandomTaskDemoSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Feb 20 2023
@author: Andy Chien
'''
class MultiArmRandomTaskDemoSM(Behavior):
	'''
	single robot arm planning for the targert pose sampled from more than one area
	'''


	def __init__(self, node):
		super(MultiArmRandomTaskDemoSM, self).__init__()
		self.name = 'Multi Arm Random Task Demo'

		# parameters of this behavior
		self.add_parameter('robot_1_ns', 'robot_1')
		self.add_parameter('robot_2_ns', 'robot_2')
		self.add_parameter('robot_3_ns', 'robot_3')
		self.add_parameter('robot_4_ns', 'robot_4')
		self.add_parameter('joint_names', dict())
		self.add_parameter('random_areas', dict())
		self.add_parameter('using_areas', dict())
		self.add_parameter('planner_RRTConnect', 'RRTConnectkConfigDefault')
		self.add_parameter('planner_AdaptPRM', 'AdaptLazyPRMkDefault')
		self.add_parameter('planner', 'BiTRRT')

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		WaitState.initialize_ros(node)
		self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r2/Single Arm Random Task Demo', node)
		self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r3/Single Arm Random Task Demo', node)
		self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r4/Single Arm Random Task Demo', node)
		self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Single Arm Random Task Demo', node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.velocity = 100

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_container_r4_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity'])

		with _sm_container_r4_0:
			# x:67 y:93
			OperatableStateMachine.add('wait',
										WaitState(wait_time=6),
										transitions={'done': 'Single Arm Random Task Demo'},
										autonomy={'done': Autonomy.Off})

			# x:205 y:197
			OperatableStateMachine.add('Single Arm Random Task Demo',
										self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r4/Single Arm Random Task Demo',
											parameters={'namespace': self.robot_4_ns, 'joint_names': self.joint_names, 'random_areas': self.random_areas, 'planner_id': self.planner, 'do_evaluation': True, 'using_areas': self.using_areas}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})


		# x:30 y:365, x:130 y:365
		_sm_container_r3_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity'])

		with _sm_container_r3_1:
			# x:106 y:80
			OperatableStateMachine.add('wait',
										WaitState(wait_time=4),
										transitions={'done': 'Single Arm Random Task Demo'},
										autonomy={'done': Autonomy.Off})

			# x:258 y:161
			OperatableStateMachine.add('Single Arm Random Task Demo',
										self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r3/Single Arm Random Task Demo',
											parameters={'namespace': self.robot_3_ns, 'joint_names': self.joint_names, 'random_areas': self.random_areas, 'planner_id': self.planner, 'do_evaluation': True, 'using_areas': self.using_areas}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})


		# x:30 y:365, x:130 y:365
		_sm_container_r2_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity'])

		with _sm_container_r2_2:
			# x:77 y:61
			OperatableStateMachine.add('wait',
										WaitState(wait_time=2),
										transitions={'done': 'Single Arm Random Task Demo'},
										autonomy={'done': Autonomy.Off})

			# x:158 y:175
			OperatableStateMachine.add('Single Arm Random Task Demo',
										self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Container_r2/Single Arm Random Task Demo',
											parameters={'namespace': self.robot_2_ns, 'joint_names': self.joint_names, 'random_areas': self.random_areas, 'planner_id': self.planner, 'do_evaluation': True, 'using_areas': self.using_areas}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})


		# x:584 y:160, x:307 y:364, x:309 y:473, x:853 y:48, x:493 y:400, x:780 y:418, x:379 y:476
		_sm_container_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['velocity'], conditions=[
										('finished', [('Single Arm Random Task Demo', 'finished'), ('Container_r2', 'finished'), ('Container_r3', 'finished'), ('Container_r4', 'finished')]),
										('failed', [('Single Arm Random Task Demo', 'failed')]),
										('failed', [('Container_r2', 'failed')]),
										('failed', [('Container_r3', 'failed')]),
										('failed', [('Container_r4', 'failed')])
										])

		with _sm_container_3:
			# x:64 y:170
			OperatableStateMachine.add('Single Arm Random Task Demo',
										self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Single Arm Random Task Demo',
											parameters={'namespace': self.robot_1_ns, 'joint_names': self.joint_names, 'random_areas': self.random_areas, 'planner_id': self.planner, 'do_evaluation': True, 'using_areas': self.using_areas}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})

			# x:638 y:36
			OperatableStateMachine.add('Container_r3',
										_sm_container_r3_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})

			# x:615 y:373
			OperatableStateMachine.add('Container_r4',
										_sm_container_r4_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})

			# x:300 y:68
			OperatableStateMachine.add('Container_r2',
										_sm_container_r2_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})



		with _state_machine:
			# x:169 y:119
			OperatableStateMachine.add('Container',
										_sm_container_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'velocity': 'velocity'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
