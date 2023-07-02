#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.decision_state import DecisionState
from task_flexbe_behaviors.move_arm_to_pose_async_sm import MoveArmToPoseAsyncSM
from task_flexbe_states.decision_by_param_state import DecisionByParam
from task_flexbe_states.hiwin_xeg32_gripper_client import HiwinXeg32GripperClient
from task_flexbe_states.set_DIO_state import SetDIOState
from task_flexbe_states.set_place_pose_random_state import SetPlacePoseRandomState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Feb 27 2022
@author: Andy Chien
'''
class MoveToPlaceSM(Behavior):
    '''
    Move robot to place object
    '''


    def __init__(self, node):
        super(MoveToPlaceSM, self).__init__()
        self.name = 'Move To Place'

        # parameters of this behavior
        self.add_parameter('io_service', dict())
        self.add_parameter('sim', True)
        self.add_parameter('place_in_random_area', False)
        self.add_parameter('joint_names', dict())
        self.add_parameter('select_tool_by_input', False)
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('place_pos_max', dict())
        self.add_parameter('place_pos_min', dict())
        self.add_parameter('vacuum_io_pins', dict())
        self.add_parameter('tool_name', 'suction')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        CheckConditionState.initialize_ros(node)
        DecisionByParam.initialize_ros(node)
        DecisionState.initialize_ros(node)
        HiwinXeg32GripperClient.initialize_ros(node)
        SetDIOState.initialize_ros(node)
        SetPlacePoseRandomState.initialize_ros(node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_3', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:1400 y:157, x:1020 y:449
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_joints', 'exe_client', 'place_pose', 'tool_name', 'velocity'], output_keys=['target_joints'])
        _state_machine.userdata.place_pos_max = self.place_pos_max
        _state_machine.userdata.place_pos_min = self.place_pos_min
        _state_machine.userdata.vacuum_io_pins = self.vacuum_io_pins
        _state_machine.userdata.vacuum_io_vals = [0]
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.translation_list = [.0, .0, 0.1]
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.target_joints = None
        _state_machine.userdata.place_pose = None
        _state_machine.userdata.tool_name = None
        _state_machine.userdata.translation_zero = [.0, .0, .0]
        _state_machine.userdata.velocity = 10

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:64 y:40
            OperatableStateMachine.add('use_random_place',
                                        DecisionByParam(decided=self.place_in_random_area, outcomes=['True', 'False']),
                                        transitions={'True': 'set_random_place_pose', 'False': 'check_trans_list'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:218 y:419
            OperatableStateMachine.add('Move Arm To Obj Up Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'wait': False}),
                                        transitions={'finished': 'Move Arm To Obj Pose Async', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'place_pose', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'target_joints': 'target_joints'})

            # x:1131 y:295
            OperatableStateMachine.add('Move Arm To Pose Async_3',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_3',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'wait': False}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'place_pose', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'target_joints': 'target_joints'})

            # x:23 y:362
            OperatableStateMachine.add('check_trans_list',
                                        CheckConditionState(predicate=lambda x: any([abs(y)>0.001 for y in x])),
                                        transitions={'true': 'Move Arm To Obj Up Pose Async', 'false': 'Move Arm To Obj Pose Async'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'translation_list'})

            # x:1146 y:69
            OperatableStateMachine.add('check_trans_list_2',
                                        CheckConditionState(predicate=lambda x: any([abs(y)>0.001 for y in x])),
                                        transitions={'true': 'Move Arm To Pose Async_3', 'false': 'finished'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'translation_list'})

            # x:793 y:238
            OperatableStateMachine.add('release_gripper',
                                        HiwinXeg32GripperClient(mode='open', direction=0, distance=0, speed=0, holding_stroke=0, holding_speed=0, holding_force=0, flag=0, namespace=self.namespace),
                                        transitions={'done': 'check_trans_list_2', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:804 y:64
            OperatableStateMachine.add('release_suction',
                                        SetDIOState(io_service=self.io_service, namespace='', sim=self.sim),
                                        transitions={'done': 'check_trans_list_2', 'failed': 'check_trans_list_2'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

            # x:459 y:146
            OperatableStateMachine.add('select_tool_depend_on_input',
                                        DecisionByParam(decided=self.select_tool_by_input, outcomes=['True', 'False']),
                                        transitions={'True': 'tool_select', 'False': 'tool_select_defult'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:262 y:39
            OperatableStateMachine.add('set_random_place_pose',
                                        SetPlacePoseRandomState(),
                                        transitions={'done': 'check_trans_list'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'place_position_max': 'place_pos_max', 'place_position_min': 'place_pos_min', 'place_pose': 'place_pose'})

            # x:504 y:239
            OperatableStateMachine.add('tool_select',
                                        DecisionState(outcomes=['pj', 'suction'], conditions=lambda x: x),
                                        transitions={'pj': 'release_gripper', 'suction': 'release_suction'},
                                        autonomy={'pj': Autonomy.Off, 'suction': Autonomy.Off},
                                        remapping={'input_value': 'tool_name'})

            # x:496 y:60
            OperatableStateMachine.add('tool_select_defult',
                                        DecisionByParam(decided=self.tool_name, outcomes=['pj', 'suction']),
                                        transitions={'pj': 'release_gripper', 'suction': 'release_suction'},
                                        autonomy={'pj': Autonomy.Off, 'suction': Autonomy.Off})

            # x:222 y:311
            OperatableStateMachine.add('Move Arm To Obj Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace}),
                                        transitions={'finished': 'select_tool_depend_on_input', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'place_pose', 'translation_list': 'translation_zero', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'target_joints': 'target_joints'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
