#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.gqcnn_grasp_plan_state import GQCNNGraspPlanState
from task_flexbe_states.img_masking_client_state import ImgMaskingClientState
from task_flexbe_states.tool_selection_state import ToolSelectionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 01 2023
@author: Andy Chien
'''
class ToolSelectionbasedonGQCNNSM(Behavior):
    '''
    Using FC pj and suc gqcnn to decide a tool and the grasp pose
    '''


    def __init__(self, node):
        super(ToolSelectionbasedonGQCNNSM, self).__init__()
        self.name = 'Tool Selection based on GQCNN'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        GQCNNGraspPlanState.initialize_ros(node)
        ImgMaskingClientState.initialize_ros(node)
        ToolSelectionState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:385 y:219, x:381 y:171, x:297 y:402
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'nothing_to_grasp'], input_keys=['curr_tool_name'], output_keys=['target_tool_name', 'target_pose'])
        _state_machine.userdata.curr_tool_name = 'suction'
        _state_machine.userdata.target_pose = None
        _state_machine.userdata.target_tool_name = ''

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:48 y:47
            OperatableStateMachine.add('get_masked_img',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=False, get_masked_img=True, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'gqcnn', 'failed': 'release_occupied_marker', 'retry': 'get_masked_img'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:45 y:292
            OperatableStateMachine.add('gqcnn',
                                        GQCNNGraspPlanState(pj_grasp_service='/gqcnn_pj/grasp_planner_segmask', suc_grasp_service='/gqcnn_suc/grasp_planner_segmask'),
                                        transitions={'done': 'select_tool', 'failed': 'release_occupied_marker', 'retry': 'release_marker_occupy', 'nothing': 'nothing_to_grasp'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off, 'nothing': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'camera_info_msg': 'img_info', 'pj_pose': 'pj_pose', 'suc_pose': 'suc_pose', 'frame': 'frame', 'pj_qv': 'pj_qv', 'suc_qv': 'suc_qv'})

            # x:195 y:118
            OperatableStateMachine.add('release_marker_occupy',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'get_masked_img', 'failed': 'failed', 'retry': 'release_marker_occupy'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:672 y:165
            OperatableStateMachine.add('release_occupied_marker',
                                        ImgMaskingClientState(namespace='', marker_id=5, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=True, get_masked_img=False, resolution_wide=516, resolution_high=386),
                                        transitions={'done': 'failed', 'failed': 'failed', 'retry': 'release_occupied_marker'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:235 y:209
            OperatableStateMachine.add('select_tool',
                                        ToolSelectionState(),
                                        transitions={'done': 'finished', 'failed': 'release_marker_occupy'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'marker_poses': 'marker_poses', 'pj_pose': 'pj_pose', 'suc_pose': 'suc_pose', 'frame': 'frame', 'pj_qv': 'pj_qv', 'suc_qv': 'suc_qv', 'curr_tool': 'curr_tool_name', 'img_info': 'img_info', 'img': 'mask_img_msg', 'target_pose': 'target_pose', 'tar_tool': 'target_tool_name'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
