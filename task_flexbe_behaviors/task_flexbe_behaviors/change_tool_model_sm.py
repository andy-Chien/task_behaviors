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
from task_flexbe_states.moveit_attached_obj_state import MoveItAttachedObjState
from task_flexbe_states.set_data_by_data_state import SetDataByDataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 03 2023
@author: Andy Chien
'''
class ChangeToolModelSM(Behavior):
    '''
    Change tool model for moveit, it will attached on tool base frame
    '''


    def __init__(self, node):
        super(ChangeToolModelSM, self).__init__()
        self.name = 'Change Tool Model'

        # parameters of this behavior
        self.add_parameter('gripper_mesh_file', dict())
        self.add_parameter('suction_mesh_file', dict())
        self.add_parameter('tool_touch_links', dict())
        self.add_parameter('namespace', '')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        CheckConditionState.initialize_ros(node)
        MoveItAttachedObjState.initialize_ros(node)
        SetDataByDataState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:807 y:153, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['curr_tool', 'target_tool'], output_keys=['tool_frame'])
        _state_machine.userdata.curr_tool = ''
        _state_machine.userdata.target_tool = 'pj'
        _state_machine.userdata.pj_frame = 'pj_tool_tip'
        _state_machine.userdata.suc_frame = 'suction_tool_tip'
        _state_machine.userdata.tool_frame = 'suction_tool_tip'
        _state_machine.userdata.none = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:45 y:113
            OperatableStateMachine.add('check_target',
                                        CheckConditionState(predicate=lambda x: 'suc' in x),
                                        transitions={'true': 'attach_suction', 'false': 'attach_griper'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'input_value': 'target_tool'})

            # x:239 y:188
            OperatableStateMachine.add('attach_suction',
                                        MoveItAttachedObjState(mesh_file=self.suction_mesh_file, operation='add', obj_type='mesh', link_name='tool_base', touch_links=self.tool_touch_links, size=None, namespace=self.namespace, obj_name='gripper_with_suction', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'detach_gripper'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'link_name': 'none', 'pose': 'none'})

            # x:474 y:194
            OperatableStateMachine.add('detach_gripper',
                                        MoveItAttachedObjState(mesh_file=self.gripper_mesh_file, operation='remove', obj_type='mesh', link_name='tool_base', touch_links=[], size=None, namespace=self.namespace, obj_name='hiwin_gripper', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'set_suc_frame'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'link_name': 'none', 'pose': 'none'})

            # x:474 y:55
            OperatableStateMachine.add('detach_suction',
                                        MoveItAttachedObjState(mesh_file=self.suction_mesh_file, operation='remove', obj_type='mesh', link_name='tool_base', touch_links=[], size=None, namespace=self.namespace, obj_name='gripper_with_suction', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'set_pf_frame'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'link_name': 'none', 'pose': 'none'})

            # x:647 y:55
            OperatableStateMachine.add('set_pf_frame',
                                        SetDataByDataState(userdata_src_names=['pj_frame'], userdata_dst_names=['tool_frame']),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'pj_frame': 'pj_frame', 'tool_frame': 'tool_frame'})

            # x:644 y:196
            OperatableStateMachine.add('set_suc_frame',
                                        SetDataByDataState(userdata_src_names=['suc_frame'], userdata_dst_names=['tool_frame']),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'suc_frame': 'suc_frame', 'tool_frame': 'tool_frame'})

            # x:240 y:61
            OperatableStateMachine.add('attach_griper',
                                        MoveItAttachedObjState(mesh_file=self.gripper_mesh_file, operation='add', obj_type='mesh', link_name='tool_base', touch_links=self.tool_touch_links, size=None, namespace=self.namespace, obj_name='hiwin_gripper', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'detach_suction'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'link_name': 'none', 'pose': 'none'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
