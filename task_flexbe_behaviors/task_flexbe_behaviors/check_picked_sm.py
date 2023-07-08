#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from task_flexbe_states.get_DIO_state import GetDIOState
from task_flexbe_states.hiwin_xeg32_gripper_client import HiwinXeg32GripperClient
from task_flexbe_states.set_DIO_state import SetDIOState
from task_flexbe_states.set_data_by_condition_state import SetDataByConditionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 08 2023
@author: Andy Chien
'''
class CheckPickedSM(Behavior):
    '''
    check it is picked or not
    '''


    def __init__(self, node):
        super(CheckPickedSM, self).__init__()
        self.name = 'Check Picked'

        # parameters of this behavior
        self.add_parameter('namespace', '')
        self.add_parameter('sim', False)
        self.add_parameter('io_topic', dict())
        self.add_parameter('pressure_sensor_pin', dict())
        self.add_parameter('gripper_sensor_pin', dict())
        self.add_parameter('io_service', dict())
        self.add_parameter('vacuum_io_pins', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DecisionState.initialize_ros(node)
        GetDIOState.initialize_ros(node)
        HiwinXeg32GripperClient.initialize_ros(node)
        SetDIOState.initialize_ros(node)
        SetDataByConditionState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:692 y:96, x:960 y:227
        _state_machine = OperatableStateMachine(outcomes=['true', 'false'], input_keys=['curr_tool_name', 'fail_cnt'], output_keys=['fail_cnt'])
        _state_machine.userdata.curr_tool_name = None
        _state_machine.userdata.pressure_sensor_pin = self.pressure_sensor_pin
        _state_machine.userdata.gripper_sensor_pin = self.gripper_sensor_pin
        _state_machine.userdata.fail_cnt = 0
        _state_machine.userdata.vacuum_io_pins = self.vacuum_io_pins
        _state_machine.userdata.vacuum_io_vals = [0]

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:52 y:124
            OperatableStateMachine.add('is_suction',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: 'suc' in x),
                                        transitions={'True': 'get_suc_io', 'False': 'get_pj_io'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
                                        remapping={'input_value': 'curr_tool_name'})

            # x:380 y:48
            OperatableStateMachine.add('check_suc_state',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: x[0]),
                                        transitions={'True': 'reset_fail_cnt', 'False': 'release_suc'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
                                        remapping={'input_value': 'pin_vals'})

            # x:818 y:122
            OperatableStateMachine.add('fail_cnt_++',
                                        SetDataByConditionState(condition=lambda x: x + 1, userdata_src_names=['fail_cnt'], userdata_dst_names=['fail_cnt']),
                                        transitions={'done': 'false'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'fail_cnt': 'fail_cnt'})

            # x:201 y:194
            OperatableStateMachine.add('get_pj_io',
                                        GetDIOState(io_topic=self.io_topic, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'check_pj_state'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'pins': 'gripper_sensor_pin', 'vals': 'pin_vals'})

            # x:207 y:45
            OperatableStateMachine.add('get_suc_io',
                                        GetDIOState(io_topic=self.io_topic, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'check_suc_state'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'pins': 'pressure_sensor_pin', 'vals': 'pin_vals'})

            # x:554 y:202
            OperatableStateMachine.add('release_gripper',
                                        HiwinXeg32GripperClient(mode='open', direction=0, distance=0, speed=0, holding_stroke=0, holding_speed=0, holding_force=0, flag=0, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'fail_cnt_++', 'failed': 'fail_cnt_++'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:562 y:39
            OperatableStateMachine.add('release_suc',
                                        SetDIOState(io_service=self.io_service, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'fail_cnt_++', 'failed': 'fail_cnt_++'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

            # x:545 y:122
            OperatableStateMachine.add('reset_fail_cnt',
                                        SetDataByConditionState(condition=lambda x: x - x, userdata_src_names=['fail_cnt'], userdata_dst_names=['fail_cnt']),
                                        transitions={'done': 'true'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'fail_cnt': 'fail_cnt'})

            # x:372 y:199
            OperatableStateMachine.add('check_pj_state',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: x[0]),
                                        transitions={'True': 'reset_fail_cnt', 'False': 'release_gripper'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
                                        remapping={'input_value': 'pin_vals'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
