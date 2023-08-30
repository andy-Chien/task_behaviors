#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_behaviors.move_arm_to_pose_async_sm import MoveArmToPoseAsyncSM
from task_flexbe_states.get_box_pointcloud_state import GetBoxPointCloudState
from task_flexbe_states.get_obj_pointcloud_state import GetObjPointCloudState
from task_flexbe_states.img_masking_client_state import ImgMaskingClientState
from task_flexbe_states.packing_plan_client import PackingPlanClient
from task_flexbe_states.set_data_by_data_state import SetDataByDataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Aug 11 2023
@author: Andy Chien
'''
class PackingPlanningSM(Behavior):
    '''
    merge obj point cloud and plan the packing pose
    '''


    def __init__(self, node):
        super(PackingPlanningSM, self).__init__()
        self.name = 'Packing Planning'

        # parameters of this behavior
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('planner', 'AdaptLazyPRM')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        GetBoxPointCloudState.initialize_ros(node)
        GetObjPointCloudState.initialize_ros(node)
        ImgMaskingClientState.initialize_ros(node)
        PackingPlanClient.initialize_ros(node)
        SetDataByDataState.initialize_ros(node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_2', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_3', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:17 y:672, x:334 y:11
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['exe_client', 'ik_target_frame'], output_keys=['exe_client', 'expected_joints', 'packing_pose'])
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.tp1 = [0.613, -0.174, 0.130, 0.131, -0.991, 0.0, 0.0]
        _state_machine.userdata.tp2 = [0.613, -0.174, 0.130, 0.131, -0.991, 0.0, 0.0]
        _state_machine.userdata.tp3 = [0.613, -0.174, 0.130, 0.131, -0.991, 0.0, 0.0]
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.ik_target_frame = None
        _state_machine.userdata.expected_joints = None
        _state_machine.userdata.translation_list = None
        _state_machine.userdata.is_first_obj = True
        _state_machine.userdata.false = False
        _state_machine.userdata.packing_pose = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:137 y:133
            OperatableStateMachine.add('Move Arm To Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner}),
                                        transitions={'finished': 'merge_point_cloud', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'tp1', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'target_joints'})

            # x:454 y:133
            OperatableStateMachine.add('Move Arm To Pose Async_2',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_2',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner}),
                                        transitions={'finished': 'merge_point_cloud', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'tp2', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'target_joints'})

            # x:779 y:130
            OperatableStateMachine.add('Move Arm To Pose Async_3',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pose Async_3',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner}),
                                        transitions={'finished': 'merge_point_cloud', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'tp3', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'expected_joints'})

            # x:265 y:451
            OperatableStateMachine.add('generate_box_pcl',
                                        GetBoxPointCloudState(),
                                        transitions={'done': 'packing_plan'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'masked_depth_img': 'mask_img_msg', 'camera_info': 'img_info', 'marker_poses': 'marker_poses', 'box_pointcloud': 'box_pointcloud', 'camera_trans_box': 'camera_trans_box', 'box_size': 'box_size', 'frame': 'frame'})

            # x:468 y:455
            OperatableStateMachine.add('get_box_img',
                                        ImgMaskingClientState(namespace='', marker_id=15, create_depth_mask=False, update_mask=False, start_update_timer=False, stop_update_timer=False, mark_release=False, get_masked_img=True, resolution_wide=512, resolution_high=512),
                                        transitions={'done': 'generate_box_pcl', 'failed': 'failed', 'retry': 'get_box_img'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'retry': Autonomy.Off},
                                        remapping={'mask_img_msg': 'mask_img_msg', 'img_info': 'img_info', 'marker_poses': 'marker_poses', 'poses_frame': 'poses_frame'})

            # x:523 y:329
            OperatableStateMachine.add('merge_point_cloud',
                                        GetObjPointCloudState(depth_camera_info='/camera/depth/camera_info', depth_image_topic='/camera/depth/image_rect_raw', tip_link='tool_tip', camera_frame='camera_depth_frame'),
                                        transitions={'done': 'get_box_img', 'failed': 'failed', 'second': 'Move Arm To Pose Async_2', 'third': 'Move Arm To Pose Async_3'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'second': Autonomy.Off, 'third': Autonomy.Off},
                                        remapping={'obj_pointcloud': 'obj_pointcloud'})

            # x:102 y:365
            OperatableStateMachine.add('packing_plan',
                                        PackingPlanClient(namespace='', frame_id='base_link'),
                                        transitions={'done': 'set_false', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'box_cloud': 'box_pointcloud', 'obj_cloud': 'obj_pointcloud', 'box_size': 'box_size', 'is_first_obj': 'is_first_obj', 'init_pose': 'tp1', 'camera_trans_box': 'camera_trans_box', 'frame': 'frame', 'packing_pose': 'packing_pose'})

            # x:85 y:525
            OperatableStateMachine.add('set_false',
                                        SetDataByDataState(userdata_src_names=['false'], userdata_dst_names=['is_first_obj']),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'false': 'false', 'is_first_obj': 'is_first_obj'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
