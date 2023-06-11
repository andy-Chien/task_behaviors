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
from task_flexbe_states.list_iterator_state import ListIteratorState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 11 2023
@author: Andy Chien
'''
class PlannerBenchmarkSM(Behavior):
    '''
    ompl_benchmarker
    '''


    def __init__(self, node):
        super(PlannerBenchmarkSM, self).__init__()
        self.name = 'Planner Benchmark'

        # parameters of this behavior
        self.add_parameter('terminal_rounds', 100)
        self.add_parameter('eval_rounds', 100)
        self.add_parameter('planner_list', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ListIteratorState.initialize_ros(node)
        self.add_behavior(MultiArmRandomTaskDemoSM, 'Multi Arm Random Task Demo', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:156 y:303, x:327 y:300
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:36 y:164
            OperatableStateMachine.add('planner_iterator',
                                        ListIteratorState(the_list=self.planner_list),
                                        transitions={'done': 'Multi Arm Random Task Demo', 'empty': 'finished'},
                                        autonomy={'done': Autonomy.Off, 'empty': Autonomy.Off},
                                        remapping={'data_out': 'planner'})

            # x:381 y:141
            OperatableStateMachine.add('Multi Arm Random Task Demo',
                                        self.use_behavior(MultiArmRandomTaskDemoSM, 'Multi Arm Random Task Demo',
                                            parameters={'terminal_rounds': self.terminal_rounds, 'eval_rounds': self.eval_rounds}),
                                        transitions={'finished': 'planner_iterator', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'planner': 'planner'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
