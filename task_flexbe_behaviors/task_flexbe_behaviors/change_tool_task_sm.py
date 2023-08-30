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
from task_flexbe_behaviors.change_tool_model_sm import ChangeToolModelSM
from task_flexbe_states.data_copy_state import DataCopyState
from task_flexbe_states.get_current_joints import GetCurrentJoints
from task_flexbe_states.hiwin_xeg32_gripper_api import HiwinXeg32GripperApi
from task_flexbe_states.moveit_async_execute_trajectory import MoveItAsyncExecuteTrajectory
from task_flexbe_states.moveit_joint_plan_state import MoveItJointsPlanState
from task_flexbe_states.moveit_wait_for_execute_state import WaitForRunningState
from task_flexbe_states.set_data_by_condition_state import SetDataByConditionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 25 2023
@author: TaiTing Tsai
'''
class ChangeToolTaskSM(Behavior):
    '''
    change endeffector tool with hiwin gripper
    '''


    def __init__(self, node):
        super(ChangeToolTaskSM, self).__init__()
        self.name = 'Change Tool Task'

        # parameters of this behavior
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('joint_names', dict())
        self.add_parameter('sim', False)
        self.add_parameter('infront_sucker', dict())
        self.add_parameter('sucker_spot', dict())
        self.add_parameter('gripper_mesh_file', dict())
        self.add_parameter('suction_mesh_file', dict())
        self.add_parameter('tool_touch_links', dict())

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DataCopyState.initialize_ros(node)
        FlexibleCheckConditionState.initialize_ros(node)
        GetCurrentJoints.initialize_ros(node)
        HiwinXeg32GripperApi.initialize_ros(node)
        MoveItAsyncExecuteTrajectory.initialize_ros(node)
        MoveItJointsPlanState.initialize_ros(node)
        SetDataByConditionState.initialize_ros(node)
        WaitForRunningState.initialize_ros(node)
        self.add_behavior(ChangeToolModelSM, 'Change Tool Model', node)
        self.add_behavior(ChangeToolModelSM, 'Change Tool Model_2', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:1298 y:714, x:1115 y:159
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['target_tool_name', 'exe_client', 'curr_tool_name'], output_keys=['curr_tool_name', 'expected_joints', 'tool_frame', 'exe_client'])
        _state_machine.userdata.direction_in = 0
        _state_machine.userdata.distance = 0
        _state_machine.userdata.speed = 6000
        _state_machine.userdata.holding_stroke = 3200
        _state_machine.userdata.holding_speed = 2000
        _state_machine.userdata.holding_force_little = 40
        _state_machine.userdata.flag = 1
        _state_machine.userdata.infront_sucker = self.infront_sucker
        _state_machine.userdata.sucker_spot = self.sucker_spot
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.planner = 'AdaptLazyPRMkDefault'
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.mode = 'open'
        _state_machine.userdata.direction_out = 1
        _state_machine.userdata.distance_little = 10
        _state_machine.userdata.speed_little = 500
        _state_machine.userdata.holding_stroke_little = 10
        _state_machine.userdata.holding_speed_little = 500
        _state_machine.userdata.holding_force = 80
        _state_machine.userdata.curr_tool_name = 'pj'
        _state_machine.userdata.target_tool_name = 'suction'
        _state_machine.userdata.tool_frame = 'suction_tool_tip'
        _state_machine.userdata.expected_joints = None

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:1071 y:17
            OperatableStateMachine.add('set_mode',
                                        SetDataByConditionState(condition=lambda x: 'open' if x == 'pj' else 'expert', userdata_src_names=['target_tool_name'], userdata_dst_names=['mode']),
                                        transitions={'done': 'check_tool'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'target_tool_name': 'target_tool_name', 'mode': 'mode'})

            # x:1138 y:356
            OperatableStateMachine.add('Change Tool Model_2',
                                        self.use_behavior(ChangeToolModelSM, 'Change Tool Model_2',
                                            parameters={'gripper_mesh_file': self.gripper_mesh_file, 'suction_mesh_file': self.suction_mesh_file, 'tool_touch_links': self.tool_touch_links, 'namespace': self.namespace}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'curr_tool': 'curr_tool_name', 'target_tool': 'target_tool_name', 'tool_frame': 'tool_frame'})

            # x:1220 y:94
            OperatableStateMachine.add('check_tool',
                                        FlexibleCheckConditionState(predicate=lambda x: x[0]==x[1], input_keys=['curr_tool', 'tar_tool']),
                                        transitions={'true': 'wait_for_running_1', 'false': 'wait_for_running_0'},
                                        autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
                                        remapping={'curr_tool': 'curr_tool_name', 'tar_tool': 'target_tool_name'})

            # x:87 y:159
            OperatableStateMachine.add('excute_paln',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'wait_for_running', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:443 y:354
            OperatableStateMachine.add('execute_plan2',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'wait_for_arrive', 'failed': 'get_curr_joint'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:279 y:279
            OperatableStateMachine.add('get_curr_joint',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'move_to_sucker_spot', 'no_msg': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'curr_joints'})

            # x:30 y:40
            OperatableStateMachine.add('get_current_joint',
                                        GetCurrentJoints(joint_names=self.joint_names, namespace=self.namespace),
                                        transitions={'done': 'move_to_sucker_front', 'no_msg': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'no_msg': Autonomy.Off},
                                        remapping={'curr_joints': 'curr_joints'})

            # x:606 y:608
            OperatableStateMachine.add('hold_or_release_2_check',
                                        HiwinXeg32GripperApi(sim=self.sim, namespace=self.namespace),
                                        transitions={'done': 'hold_or_release_3_check', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'mode': 'mode', 'direction': 'direction_out', 'distance': 'distance_little', 'speed': 'speed_little', 'holding_stroke': 'holding_stroke_little', 'holding_speed': 'holding_speed_little', 'holding_force': 'holding_force_little', 'flag': 'flag'})

            # x:413 y:723
            OperatableStateMachine.add('hold_or_release_3_check',
                                        HiwinXeg32GripperApi(sim=self.sim, namespace=self.namespace),
                                        transitions={'done': 'set_curr_name', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'mode': 'mode', 'direction': 'direction_in', 'distance': 'distance_little', 'speed': 'speed_little', 'holding_stroke': 'holding_stroke_little', 'holding_speed': 'holding_speed_little', 'holding_force': 'holding_force', 'flag': 'flag'})

            # x:48 y:551
            OperatableStateMachine.add('hold_or_release_sucker',
                                        HiwinXeg32GripperApi(sim=self.sim, namespace=self.namespace),
                                        transitions={'done': 'Change Tool Model', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'mode': 'mode', 'direction': 'direction_in', 'distance': 'distance', 'speed': 'speed', 'holding_stroke': 'holding_stroke', 'holding_speed': 'holding_speed', 'holding_force': 'holding_force', 'flag': 'flag'})

            # x:678 y:400
            OperatableStateMachine.add('move_back',
                                        MoveItAsyncExecuteTrajectory(group_name=self.group_name, namespace=self.namespace),
                                        transitions={'done': 'wait_for_arm', 'failed': 'plan_back'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'joint_trajectory': 'joint_trajectory', 'exe_client': 'exe_client'})

            # x:282 y:119
            OperatableStateMachine.add('move_to_sucker_front',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'excute_paln', 'retriable': 'move_to_sucker_front'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'curr_joints', 'target_joints': 'infront_sucker', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:28 y:419
            OperatableStateMachine.add('move_to_sucker_spot',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'execute_plan2', 'retriable': 'move_to_sucker_spot'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'curr_joints', 'target_joints': 'sucker_spot', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:253 y:610
            OperatableStateMachine.add('plan_back',
                                        MoveItJointsPlanState(group_name=self.group_name, joint_names=self.joint_names, retry_cnt=5, namespace=self.namespace, planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10),
                                        transitions={'failed': 'failed', 'done': 'move_back', 'retriable': 'plan_back'},
                                        autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off, 'retriable': Autonomy.Off},
                                        remapping={'start_joints': 'sucker_spot', 'target_joints': 'infront_sucker', 'velocity': 'velocity', 'planner': 'planner', 'joint_trajectory': 'joint_trajectory', 'planning_time': 'planning_time', 'planning_error_code': 'planning_error_code'})

            # x:979 y:577
            OperatableStateMachine.add('set_curr_name',
                                        DataCopyState(),
                                        transitions={'done': 'ste_expected_joints'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data_in': 'target_tool_name', 'data_out': 'curr_tool_name'})

            # x:952 y:706
            OperatableStateMachine.add('ste_expected_joints',
                                        DataCopyState(),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'data_in': 'infront_sucker', 'data_out': 'expected_joints'})

            # x:809 y:511
            OperatableStateMachine.add('wait_for_arm',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_arm', 'done': 'hold_or_release_2_check', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:275 y:495
            OperatableStateMachine.add('wait_for_arrive',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_arrive', 'done': 'hold_or_release_sucker', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:0 y:277
            OperatableStateMachine.add('wait_for_running',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running', 'done': 'get_curr_joint', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:757 y:55
            OperatableStateMachine.add('wait_for_running_0',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running_0', 'done': 'get_current_joint', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:1296 y:197
            OperatableStateMachine.add('wait_for_running_1',
                                        WaitForRunningState(wait_until_complete_rate=0, wait_until_points_left=0, namespace=self.namespace),
                                        transitions={'waiting': 'wait_for_running_1', 'done': 'Change Tool Model_2', 'collision': 'failed', 'failed': 'failed'},
                                        autonomy={'waiting': Autonomy.Off, 'done': Autonomy.Off, 'collision': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:70 y:688
            OperatableStateMachine.add('Change Tool Model',
                                        self.use_behavior(ChangeToolModelSM, 'Change Tool Model',
                                            parameters={'gripper_mesh_file': self.gripper_mesh_file, 'suction_mesh_file': self.suction_mesh_file, 'tool_touch_links': self.tool_touch_links, 'namespace': self.namespace}),
                                        transitions={'finished': 'plan_back', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'curr_tool': 'curr_tool_name', 'target_tool': 'target_tool_name', 'tool_frame': 'tool_frame'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
