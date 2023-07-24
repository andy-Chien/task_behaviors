#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.hiwin_xeg32_gripper_api import HiwinXeg32GripperApi
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 12 2023
@author: TaiTing Tsai
'''
class TestGripperSM(Behavior):
    '''
    test hiwin xeg32 gripper api
    '''


    def __init__(self, node):
        super(TestGripperSM, self).__init__()
        self.name = 'Test Gripper'

        # parameters of this behavior
        self.add_parameter('namespace', '')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        HiwinXeg32GripperApi.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:879 y:371, x:457 y:119
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['gripper_mode'])
        _state_machine.userdata.direction = 0
        _state_machine.userdata.distance = 0
        _state_machine.userdata.speed = 6000
        _state_machine.userdata.holding_stroke = 3000
        _state_machine.userdata.holding_speed = 2000
        _state_machine.userdata.holding_force = 40
        _state_machine.userdata.flag = 1
        _state_machine.userdata.close = 'close'
        _state_machine.userdata.open = 'open'
        _state_machine.userdata.expert = 'expert'
        _state_machine.userdata.off = 'off'
        _state_machine.userdata.gripper_mode = 'open'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:376 y:293
            OperatableStateMachine.add('hiwin2',
                                        HiwinXeg32GripperApi(sim=False, namespace=self.namespace),
                                        transitions={'done': 'finished', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'mode': 'gripper_mode', 'direction': 'direction', 'distance': 'distance', 'speed': 'speed', 'holding_stroke': 'holding_stroke', 'holding_speed': 'holding_speed', 'holding_force': 'holding_force', 'flag': 'flag'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
