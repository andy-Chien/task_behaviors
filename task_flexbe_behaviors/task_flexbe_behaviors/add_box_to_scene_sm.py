#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.compute_box_pose_state import ComputeBoxPoseState
from task_flexbe_states.moveit_collision_obj_state import MoveItCollisionObjState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 06 2023
@author: Andy Chien
'''
class AddBoxToSceneSM(Behavior):
    '''
    As the name
    '''


    def __init__(self, node):
        super(AddBoxToSceneSM, self).__init__()
        self.name = 'Add Box To Scene'

        # parameters of this behavior
        self.add_parameter('box_mesh', dict())
        self.add_parameter('box_ring_mesh', dict())
        self.add_parameter('namespace', '')
        self.add_parameter('box_name', '')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ComputeBoxPoseState.initialize_ros(node)
        MoveItCollisionObjState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:614 y:282
        _state_machine = OperatableStateMachine(outcomes=['finished'], input_keys=['frame_id', 'marker_poses'])
        _state_machine.userdata.frame_id = ''
        _state_machine.userdata.marker_poses = []

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:77 y:122
            OperatableStateMachine.add('compute_box_pose',
                                        ComputeBoxPoseState(),
                                        transitions={'done': 'add_box'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'marker_poses': 'marker_poses', 'box_pose': 'box_pose'})

            # x:498 y:121
            OperatableStateMachine.add('add_box_ring',
                                        MoveItCollisionObjState(mesh_file=self.box_ring_mesh, operation='add', obj_type='mesh', frame_id='', size=None, namespace=self.namespace, obj_name=self.box_name + '_ring', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'frame_id': 'frame_id', 'pose': 'box_pose'})

            # x:289 y:123
            OperatableStateMachine.add('add_box',
                                        MoveItCollisionObjState(mesh_file=self.box_mesh, operation='add', obj_type='mesh', frame_id='', size=None, namespace=self.namespace, obj_name=self.box_name, pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]),
                                        transitions={'done': 'add_box_ring'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'frame_id': 'frame_id', 'pose': 'box_pose'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
