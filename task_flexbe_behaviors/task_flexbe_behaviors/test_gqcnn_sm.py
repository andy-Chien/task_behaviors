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
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
from task_flexbe_states.img_masking_client_state import ImgMaskingClientState
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
        GQCNNGraspPlanState.initialize_ros(node)
        ImgMaskingClientState.initialize_ros(node)
        WaitState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:610 y:464, x:156 y:220
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:107 y:30
            OperatableStateMachine.add('init_bin_mask',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=True, update_mask=True, start_update_timer=False, stop_update_timer=False, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'finished', 'failed': 'failed', 'retry': 'init_bin_mask'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:510 y:295
            OperatableStateMachine.add('gqcnn',
                                        GQCNNGraspPlanState(pj_grasp_service='/gqcnn_pj/grasp_planner_segmask', suc_grasp_service='/gqcnn_suc/grasp_planner_segmask'),
                                        transitions={'done': 'release_marker_occupy_and_stop_timmer', 'failed': 'failed', 'retry': 'release_marker_occupy', 'nothing': 'finished'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off, 'nothing': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'camera_info_msg': 'img_info', 'pj_pose': 'pj_pose', 'suc_pose': 'suc_pose', 'frame': 'frame', 'pj_qv': 'pj_qv', 'suc_qv': 'suc_qv'})

            # x:358 y:206
            OperatableStateMachine.add('release_marker_occupy',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'get_masked_img', 'failed': 'failed', 'retry': 'release_marker_occupy'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:176 y:373
            OperatableStateMachine.add('release_marker_occupy_and_stop_timmer',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=True, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'finished', 'failed': 'failed', 'retry': 'release_marker_occupy_and_stop_timmer'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:513 y:25
            OperatableStateMachine.add('start_masking_timer',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=True, stop_update_timer=False, mark_release=False, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'get_masked_img', 'failed': 'failed', 'retry': 'start_masking_timer'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:299 y:32
            OperatableStateMachine.add('wait',
                                        WaitState(wait_time=0.1),
                                        transitions={'done': 'start_masking_timer'},
                                        autonomy={'done': Autonomy.Off})

            # x:516 y:127
            OperatableStateMachine.add('get_masked_img',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=False, get_masked_img=True, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'gqcnn', 'failed': 'failed', 'retry': 'get_masked_img'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
