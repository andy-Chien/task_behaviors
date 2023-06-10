#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_behaviors.multi_arm_random_task_demo_sm import MultiArmRandomTaskDemoSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 09 2023
@author: TaiTing Tsai
'''
class FourArmsPlannerBenchmarkSM(Behavior):
	'''
	ompl_benchmarker
	'''


	def __init__(self, node):
		super(FourArmsPlannerBenchmarkSM, self).__init__()
		self.name = 'Four Arms Planner Benchmark'

		# parameters of this behavior
		self.add_parameter('terminal_rounds', 100)
		self.add_parameter('eval_rounds', 100)

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)

		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_2/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_3/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_4/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_5/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_6/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_7/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Container_8/Multi Arm Random Task Demo', node)
		self.add_behavior(MultiArmRandomTaskDemoSM, 'Multi Arm Random Task Demo', node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:286 y:665, x:184 y:393
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_container_8_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_8_0:
			# x:217 y:174
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_8/Multi Arm Random Task Demo',
											parameters={'planner': "RRTConnectkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_7_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_7_1:
			# x:70 y:142
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_7/Multi Arm Random Task Demo',
											parameters={'planner': "PRMkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_6_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_6_2:
			# x:93 y:119
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_6/Multi Arm Random Task Demo',
											parameters={'planner': "LazyPRMkDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_5_3 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_5_3:
			# x:37 y:107
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_5/Multi Arm Random Task Demo',
											parameters={'planner': "TRRTkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_4_4 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_4_4:
			# x:145 y:146
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_4/Multi Arm Random Task Demo',
											parameters={'planner': "RRTkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_3_5 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_3_5:
			# x:96 y:133
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_3/Multi Arm Random Task Demo',
											parameters={'planner': "LBKPIECEkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_2_6 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_2_6:
			# x:42 y:92
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container_2/Multi Arm Random Task Demo',
											parameters={'planner': "SBLkConfigDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_container_7 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_7:
			# x:118 y:180
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Container/Multi Arm Random Task Demo',
											parameters={'planner': "AdaptLazyPRMkDefault"}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:34 y:73
			OperatableStateMachine.add('Multi Arm Random Task Demo',
										self.use_behavior(MultiArmRandomTaskDemoSM, 'Multi Arm Random Task Demo'),
										transitions={'finished': 'Container', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:449 y:158
			OperatableStateMachine.add('Container_2',
										_sm_container_2_6,
										transitions={'finished': 'Container_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:566 y:248
			OperatableStateMachine.add('Container_3',
										_sm_container_3_5,
										transitions={'finished': 'Container_4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:685 y:339
			OperatableStateMachine.add('Container_4',
										_sm_container_4_4,
										transitions={'finished': 'Container_5', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:800 y:428
			OperatableStateMachine.add('Container_5',
										_sm_container_5_3,
										transitions={'finished': 'Container_6', 'failed': 'Container_5'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:687 y:504
			OperatableStateMachine.add('Container_6',
										_sm_container_6_2,
										transitions={'finished': 'Container_7', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:563 y:578
			OperatableStateMachine.add('Container_7',
										_sm_container_7_1,
										transitions={'finished': 'Container_8', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:445 y:639
			OperatableStateMachine.add('Container_8',
										_sm_container_8_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:326 y:83
			OperatableStateMachine.add('Container',
										_sm_container_7,
										transitions={'finished': 'Container_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
