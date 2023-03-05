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
        self.add_parameter('planner_RRTConnect', 'RRTConnectkConfigDefault')
        self.add_parameter('planner_AdaptPRM', 'AdaptPRMkDefault')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        WaitState.initialize_ros(node)
        self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Container/Single Arm Random Task Demo', node)
        self.add_behavior(SingleArmRandomTaskDemoSM, 'Container/Single Arm Random Task Demo', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.velocity = 100
        _state_machine.userdata.exe_client = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

        # x:30 y:365, x:130 y:365
        _sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity', 'exe_client'])

        with _sm_container_0:
            # x:77 y:61
            OperatableStateMachine.add('wai',
                                        WaitState(wait_time=3),
                                        transitions={'done': 'Single Arm Random Task Demo'},
                                        autonomy={'done': Autonomy.Off})

            # x:158 y:175
            OperatableStateMachine.add('Single Arm Random Task Demo',
                                        self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Container/Single Arm Random Task Demo',
                                            parameters={'namespace': self.robot_2_ns, 'planner_id': self.planner_AdaptPRM}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


        # x:336 y:194, x:307 y:364, x:326 y:111, x:356 y:365, x:430 y:365
        _sm_container_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['velocity', 'exe_client'], conditions=[
                                        ('finished', [('Single Arm Random Task Demo', 'finished')]),
                                        ('finished', [('Container', 'finished')]),
                                        ('failed', [('Single Arm Random Task Demo', 'failed'), ('Container', 'failed')])
                                        ])

        with _sm_container_1:
            # x:64 y:170
            OperatableStateMachine.add('Single Arm Random Task Demo',
                                        self.use_behavior(SingleArmRandomTaskDemoSM, 'Container/Single Arm Random Task Demo',
                                            parameters={'namespace': self.robot_1_ns, 'planner_id': self.planner_AdaptPRM, 'do_evaluation': True}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

            # x:461 y:173
            OperatableStateMachine.add('Container',
                                        _sm_container_0,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'velocity': 'velocity', 'exe_client': 'exe_client'})



        with _state_machine:
            # x:169 y:119
            OperatableStateMachine.add('Container',
                                        _sm_container_1,
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'velocity': 'velocity', 'exe_client': 'exe_client'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
