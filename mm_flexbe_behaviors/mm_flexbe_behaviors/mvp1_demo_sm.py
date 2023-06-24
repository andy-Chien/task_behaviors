#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from mm_flexbe_behaviors.move_arm_to_pose_sm import MoveArmToPoseSM
from mm_flexbe_behaviors.move_base_to_pose_sm import MoveBaseToPoseSM
from mm_flexbe_behaviors.move_mm_to_pose_sm import MoveMMToPoseSM
from mm_flexbe_states.get_capture_pose import GetCapturePose
from mm_flexbe_states.get_painting_params import GetPaintingParams
from mm_flexbe_states.painting_path_planning import PaintingPathPlanning
from mm_flexbe_states.path_tracking_state import PathTrackingState
from mm_flexbe_states.pose_to_base_and_arm import PoseToBaseAndArm
from mm_flexbe_states.switch_controller import SwitchControllers
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 13 2023
@author: Andy Chien
'''
class Mvp1DemoSM(Behavior):
    '''
    Behavior for mvp1 demonstraction
    '''


    def __init__(self, node):
        super(Mvp1DemoSM, self).__init__()
        self.name = 'Mvp1 Demo'

        # parameters of this behavior
        self.add_parameter('painting_areas', dict())
        self.add_parameter('distance_from_wall', 1.5)
        self.add_parameter('image_paths', dict())
        self.add_parameter('area_index_of_image', dict())
        self.add_parameter('namespace', '')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        GetCapturePose.initialize_ros(node)
        GetPaintingParams.initialize_ros(node)
        PaintingPathPlanning.initialize_ros(node)
        PathTrackingState.initialize_ros(node)
        PoseToBaseAndArm.initialize_ros(node)
        SwitchControllers.initialize_ros(node)
        self.add_behavior(MoveArmToPoseSM, 'Move Arm To Pose', node)
        self.add_behavior(MoveBaseToPoseSM, 'Move Base To Pose', node)
        self.add_behavior(MoveMMToPoseSM, 'Move MM To Pose', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:573 y:405, x:340 y:268
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.mm_pose = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


        with _state_machine:
            # x:79 y:26
            OperatableStateMachine.add('get params',
                                        GetPaintingParams(painting_areas=self.painting_areas, image_paths=self.image_paths, area_index_of_image=self.area_index_of_image),
                                        transitions={'done': 'get_first_capture_pose', 'finish': 'finished'},
                                        autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
                                        remapping={'painting_area': 'painting_area', 'image_path': 'image_path'})

            # x:53 y:148
            OperatableStateMachine.add('Move Base To Pose',
                                        self.use_behavior(MoveBaseToPoseSM, 'Move Base To Pose'),
                                        transitions={'finished': 'Move Arm To Pose', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'base_pose'})

            # x:319 y:381
            OperatableStateMachine.add('Move MM To Pose',
                                        self.use_behavior(MoveMMToPoseSM, 'Move MM To Pose'),
                                        transitions={'finished': 'get_capture_pose', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'velocity': 'velocity', 'mm_pose': 'capture_pose'})

            # x:272 y:150
            OperatableStateMachine.add('convert_pose',
                                        PoseToBaseAndArm(),
                                        transitions={'done': 'Move Base To Pose'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'mm_pose': 'capture_pose', 'base_pose': 'base_pose', 'arm_pose': 'arm_pose'})

            # x:69 y:371
            OperatableStateMachine.add('get_capture_pose',
                                        GetCapturePose(namespace=self.namespace, init=False),
                                        transitions={'failed': 'failed', 'next': 'Move MM To Pose', 'finish': 'switch_controller'},
                                        autonomy={'failed': Autonomy.Off, 'next': Autonomy.Off, 'finish': Autonomy.Off},
                                        remapping={'painting_area': 'painting_area', 'capture_pose': 'capture_pose'})

            # x:256 y:16
            OperatableStateMachine.add('get_first_capture_pose',
                                        GetCapturePose(namespace=self.namespace, init=True),
                                        transitions={'failed': 'failed', 'next': 'sw1', 'finish': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'next': Autonomy.Off, 'finish': Autonomy.Off},
                                        remapping={'painting_area': 'painting_area', 'capture_pose': 'capture_pose'})

            # x:496 y:484
            OperatableStateMachine.add('painting',
                                        PathTrackingState(namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'finished'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'painting_path': 'painting_path'})

            # x:243 y:484
            OperatableStateMachine.add('painting_plan',
                                        PaintingPathPlanning(namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'painting'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off},
                                        remapping={'image_path': 'image_path', 'painting_path': 'painting_path'})

            # x:468 y:20
            OperatableStateMachine.add('sw1',
                                        SwitchControllers(active_controllers=['mm_servo_controller'], deactive_controllers=['mm_trajectory_controller'], namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'sw2'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:635 y:29
            OperatableStateMachine.add('sw2',
                                        SwitchControllers(active_controllers=['mm_trajectory_controller'], deactive_controllers=['mm_servo_controller'], namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'convert_pose'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:63 y:482
            OperatableStateMachine.add('switch_controller',
                                        SwitchControllers(active_controllers=['mm_servo_controller'], deactive_controllers=['mm_trajectory_controller'], namespace=self.namespace),
                                        transitions={'failed': 'failed', 'done': 'painting_plan'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

            # x:52 y:259
            OperatableStateMachine.add('Move Arm To Pose',
                                        self.use_behavior(MoveArmToPoseSM, 'Move Arm To Pose'),
                                        transitions={'finished': 'get_capture_pose', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'arm_pose'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
