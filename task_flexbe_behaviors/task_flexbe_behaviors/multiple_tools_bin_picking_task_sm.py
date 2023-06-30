#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.flexible_check_condition_state import FlexibleCheckConditionState
from task_flexbe_behaviors.change_tool_task_sm import ChangeToolTaskSM
from task_flexbe_behaviors.move_arm_to_pose_async_sm import MoveArmToPoseAsyncSM
from task_flexbe_behaviors.move_to_pick_sm import MoveToPickSM
from task_flexbe_behaviors.move_to_place_sm import MoveToPlaceSM
from task_flexbe_behaviors.tool_selection_based_on_gqcnn_sm import ToolSelectionbasedonGQCNNSM
from task_flexbe_states.img_masking_client_state import ImgMaskingClientState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 01 2023
@author: Andy Chien
'''
class MultipleToolsBinPickingTaskSM(Behavior):
    '''
    Bin picking using multiple tools depend on GQCNN
    '''


    def __init__(self, node):
        super(MultipleToolsBinPickingTaskSM, self).__init__()
        self.name = 'Multiple Tools Bin Picking Task'

        # parameters of this behavior
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('namespace', '')
        self.add_parameter('sim', False)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        FlexibleCheckConditionState.initialize_ros(node)
        ImgMaskingClientState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)
        self.add_behavior(ChangeToolTaskSM, 'Change Tool Task', node)
        self.add_behavior(ChangeToolTaskSM, 'Change Tool Task_2', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Init Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Init Pose Async 2', node)
        self.add_behavior(MoveToPickSM, 'Move To Pick', node)
        self.add_behavior(MoveToPlaceSM, 'Move To Place', node)
        self.add_behavior(ToolSelectionbasedonGQCNNSM, 'Tool Selection based on GQCNN', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:600 y:213, x:108 y:555
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.init_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        _state_machine.userdata.zero_vector_3 = [0.0, 0.0, 0.0]
        _state_machine.userdata.gripper_mode = 'expert'
        _state_machine.userdata.vacuum_io_pins = [0]
        _state_machine.userdata.pick_pose = None
        _state_machine.userdata.pressure_sensor_pin = [2]
        _state_machine.userdata.curr_tool_name = 'pj'
        _state_machine.userdata.place_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.target_tool_name = 'suction'
        _state_machine.userdata.exe_client = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:64 y:35
            OperatableStateMachine.add('Move Arm To Init Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Init Pose Async',
                                            default_keys=['start_joints'],
                                            parameters={'group_name': self.group_name, 'namespace': self.namespace, 'use_curr_as_start': True}),
                                        transitions={'finished': 'start_img_update', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'init_pose', 'translation_list': 'zero_vector_3', 'velocity': 'velocity', 'target_joints': 'expected_joints'})

            # x:522 y:691
            OperatableStateMachine.add('Change Tool Task_2',
                                        self.use_behavior(ChangeToolTaskSM, 'Change Tool Task_2',
                                            parameters={'namespace': self.namespace, 'group_name': self.group_name, 'sim': self.sim}),
                                        transitions={'finished': 'Move To Pick', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_tool_name': 'target_tool_name', 'curr_tool_name': 'curr_tool_name', 'infront_sucker': 'expected_joints'})

            # x:512 y:263
            OperatableStateMachine.add('Move Arm To Init Pose Async 2',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Init Pose Async 2',
                                            parameters={'group_name': self.group_name, 'namespace': self.namespace, 'wait': False}),
                                        transitions={'finished': 'Tool Selection based on GQCNN', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'init_pose', 'translation_list': 'zero_vector_3', 'start_joints': 'expected_joints', 'velocity': 'velocity', 'target_joints': 'expected_joints'})

            # x:531 y:588
            OperatableStateMachine.add('Move To Pick',
                                        self.use_behavior(MoveToPickSM, 'Move To Pick',
                                            default_keys=['vacuum_io_vals'],
                                            parameters={'sim': self.sim, 'select_tool_by_input': True, 'namespace': self.namespace, 'group_name': self.group_name}),
                                        transitions={'finished': 'release_occupied_marker', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'vacuum_io_pins': 'vacuum_io_pins', 'start_joints': 'expected_joints', 'pick_pose': 'pick_pose', 'pick_pose': 'pick_pose', 'pressure_sensor_pin': 'pressure_sensor_pin', 'velocity': 'velocity', 'target_joints': 'expected_joints'})

            # x:534 y:374
            OperatableStateMachine.add('Move To Place',
                                        self.use_behavior(MoveToPlaceSM, 'Move To Place',
                                            default_keys=['vacuum_io_vals','place_pos_max','place_pos_min'],
                                            parameters={'sim': self.sim, 'place_in_random_area': True, 'select_tool_by_input': True, 'namespace': self.namespace, 'group_name': self.group_name}),
                                        transitions={'finished': 'Move Arm To Init Pose Async 2', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'vacuum_io_pins': 'vacuum_io_pins', 'start_joints': 'expected_joints', 'place_pose': 'place_pose', 'tool_name': 'curr_tool_name', 'velocity': 'velocity', 'target_joints': 'expected_joints'})

            # x:787 y:261
            OperatableStateMachine.add('Tool Selection based on GQCNN',
                                        self.use_behavior(ToolSelectionbasedonGQCNNSM, 'Tool Selection based on GQCNN'),
                                        transitions={'finished': 'check_tool', 'failed': 'stop_img_update'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'curr_tool_name': 'curr_tool_name', 'target_tool_name': 'target_tool_name', 'target_pose': 'target_pose'})

            # x:775 y:687
            OperatableStateMachine.add('check_tool',
                                        FlexibleCheckConditionState(predicate=lambda x: x[0] == x[1], input_keys=['curr_tool', 'target_tool']),
                                        transitions={'true': 'Move To Pick', 'false': 'wait_for_robot'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'curr_tool': 'curr_tool_name', 'target_tool': 'target_tool_name'})

            # x:530 y:469
            OperatableStateMachine.add('release_occupied_marker',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'Move To Place', 'failed': 'release_occupied_marker', 'retry': 'release_occupied_marker'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses'})

            # x:331 y:35
            OperatableStateMachine.add('start_img_update',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=True, start_update_timer=True, stop_update_timer=False, mark_release=False, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'Change Tool Task', 'failed': 'failed', 'retry': 'start_img_update'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses'})

            # x:543 y:116
            OperatableStateMachine.add('stop_img_update',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=True, mark_release=False, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'finished', 'failed': 'failed', 'retry': 'stop_img_update'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses'})

            # x:532 y:785
            OperatableStateMachine.add('wait_for_robot',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_robot', 'done': 'Change Tool Task_2', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:543 y:29
            OperatableStateMachine.add('Change Tool Task',
                                        self.use_behavior(ChangeToolTaskSM, 'Change Tool Task',
                                            parameters={'namespace': self.namespace, 'group_name': self.group_name, 'sim': self.sim}),
                                        transitions={'finished': 'Tool Selection based on GQCNN', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_tool_name': 'target_tool_name', 'curr_tool_name': 'curr_tool_name', 'infront_sucker': 'expected_joints'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
