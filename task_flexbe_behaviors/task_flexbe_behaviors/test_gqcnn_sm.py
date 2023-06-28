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
from task_flexbe_states.create_depth_mask_state import CreateDepthMask
from task_flexbe_states.get_mask_image_state import GetMaskImageState
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 14 2023
@author: TaiTing Tsai
'''
class TestGQCNNSM(Behavior):
    '''
    test gqcnn
    '''


    def __init__(self, node):
        super(TestGQCNNSM, self).__init__()
        self.name = 'Test GQCNN'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        CreateDepthMask.initialize_ros(node)
        GQCNNGraspPlanState.initialize_ros(node)
        GetMaskImageState.initialize_ros(node)
        WaitState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:525 y:402, x:143 y:220
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:527 y:74
            OperatableStateMachine.add('mask_image',
                                        GetMaskImageState(namespace='', marker_id=5, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'gqcnn', 'failed': 'failed', 'retry': 'mask_image'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info'})

            # x:514 y:260
            OperatableStateMachine.add('gqcnn',
                                        GQCNNGraspPlanState(grasp_service='/gqcnn/grasp_planner_segmask', depth_camera_info='/depth_to_rgb/camera_info'),
                                        transitions={'done': 'finished', 'failed': 'failed', 'retry': 'mask_image'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'camera_info_msg': 'img_info', 'grasp_pos': 'grasp_pos'})

            # x:325 y:22
            OperatableStateMachine.add('wait',
                                        WaitState(wait_time=30),
                                        transitions={'done': 'mask_image'},
                                        autonomy={'done': Autonomy.Off})

            # x:93 y:26
            OperatableStateMachine.add('create_depth_mask',
                                        CreateDepthMask(namespace='', marker_id=5),
                                        transitions={'done': 'wait', 'failed': 'failed', 'retry': 'create_depth_mask'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
