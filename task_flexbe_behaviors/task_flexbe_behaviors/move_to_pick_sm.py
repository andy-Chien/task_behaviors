#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from task_flexbe_behaviors.move_arm_to_pose_async_sm import MoveArmToPoseAsyncSM
from task_flexbe_states.decision_by_param_state import DecisionByParam
from task_flexbe_states.get_DIO_state import GetDIOState
from task_flexbe_states.hiwin_xeg32_gripper_api import HiwinXeg32GripperApi
from task_flexbe_states.moveit_canccel_execute_state import MoveitCancelExecuteState
from task_flexbe_states.set_DIO_state import SetDIOState
from task_flexbe_states.set_data_by_condition_state import SetDataByConditionState
from task_flexbe_states.set_data_by_data_state import SetDataByDataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 30 2023
@author: Andy Chien
'''
class MoveToPickSM(Behavior):
    '''
    Move robot to Pick object
    '''


    def __init__(self, node):
        super(MoveToPickSM, self).__init__()
        self.name = 'Move To Pick'

        # parameters of this behavior
        self.add_parameter('io_service', dict())
        self.add_parameter('sim', True)
        self.add_parameter('joint_names', dict())
        self.add_parameter('select_tool_by_input', False)
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')
        self.add_parameter('vacuum_io_pins', dict())
        self.add_parameter('pressure_sensor_pin', dict())
        self.add_parameter('io_topic', dict())
        self.add_parameter('tool_name', 'suction')
        self.add_parameter('planner', 'BiTRRT')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DecisionByParam.initialize_ros(node)
        DecisionState.initialize_ros(node)
        GetDIOState.initialize_ros(node)
        HiwinXeg32GripperApi.initialize_ros(node)
        MoveitCancelExecuteState.initialize_ros(node)
        SetDIOState.initialize_ros(node)
        SetDataByConditionState.initialize_ros(node)
        SetDataByDataState.initialize_ros(node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async 2', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pick Pose Front Async', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:1041 y:339, x:765 y:293
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['start_joints', 'velocity', 'exe_client', 'pick_pose', 'ik_target_frame', 'tool_name'], output_keys=['expected_joints', 'exe_client'])
        _state_machine.userdata.vacuum_io_pins = self.vacuum_io_pins
        _state_machine.userdata.vacuum_io_vals = [1]
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.translation_list = [.0, .0, -0.05]
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.expected_joints = None
        _state_machine.userdata.pick_pose = None
        _state_machine.userdata.tool_name = None
        _state_machine.userdata.translation_zero = [.0, .0, .0]
        _state_machine.userdata.pressure_sensor_pin = self.pressure_sensor_pin
        _state_machine.userdata.velocity = 10
        _state_machine.userdata.ik_target_frame = 'tool_tip'
        _state_machine.userdata.translation_z_up_list = [.0, .0, -0.2]
        _state_machine.userdata.check_suc_cnt = 0
        _state_machine.userdata.zero = 0
        _state_machine.userdata.speed = 6000
        _state_machine.userdata.holding_stroke = 3000
        _state_machine.userdata.holding_speed = 2000
        _state_machine.userdata.holding_force = 40
        _state_machine.userdata.flag = 1
        _state_machine.userdata.mode = 'expert'

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:32 y:285
            OperatableStateMachine.add('reset_cnt',
                                        SetDataByDataState(userdata_src_names=['zero'], userdata_dst_names=['check_suc_cnt']),
                                        transitions={'done': 'Move Arm To Pick Pose Front Async'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'zero': 'zero', 'check_suc_cnt': 'check_suc_cnt'})

            # x:521 y:215
            OperatableStateMachine.add('Move Arm To Obj Pose Async 2',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async 2',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner, 'wait': False}),
                                        transitions={'finished': 'suc', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_zero', 'start_joints': 'expected_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'expected_joints'})

            # x:854 y:228
            OperatableStateMachine.add('Move Arm To Obj Up Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner, 'wait': False, 'translation_in_target_frame': False}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_z_up_list', 'start_joints': 'expected_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'expected_joints'})

            # x:147 y:401
            OperatableStateMachine.add('Move Arm To Pick Pose Front Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Pick Pose Front Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner, 'wait': False}),
                                        transitions={'finished': 'select_tool_depend_on_input', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'expected_joints'})

            # x:907 y:409
            OperatableStateMachine.add('cancel_execution',
                                        MoveitCancelExecuteState(namespace=self.namespace, sim=False),
                                        transitions={'done': 'Move Arm To Obj Up Async'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:551 y:419
            OperatableStateMachine.add('check_suc',
                                        GetDIOState(io_topic=self.io_topic, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'succked'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'pins': 'pressure_sensor_pin', 'vals': 'pressure_sensor_val'})

            # x:702 y:507
            OperatableStateMachine.add('cnt++',
                                        SetDataByConditionState(condition=lambda x: x + 1, userdata_src_names=['check_suc_cnt'], userdata_dst_names=['check_suc_cnt']),
                                        transitions={'done': 'giveup'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'check_suc_cnt': 'check_suc_cnt'})

            # x:696 y:603
            OperatableStateMachine.add('giveup',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: x > 20),
                                        transitions={'True': 'Move Arm To Obj Up Async', 'False': 'check_suc'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
                                        remapping={'input_value': 'check_suc_cnt'})

            # x:910 y:96
            OperatableStateMachine.add('hiwin',
                                        HiwinXeg32GripperApi(sim=self.sim, namespace=self.namespace),
                                        transitions={'done': 'Move Arm To Obj Up Async', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'mode': 'mode', 'direction': 'zero', 'distance': 'zero', 'speed': 'speed', 'holding_stroke': 'holding_stroke', 'holding_speed': 'holding_speed', 'holding_force': 'holding_force', 'flag': 'flag'})

            # x:124 y:163
            OperatableStateMachine.add('select_tool_depend_on_input',
                                        DecisionByParam(decided=self.select_tool_by_input, outcomes=['True', 'False']),
                                        transitions={'True': 'tool_select', 'False': 'tool_select_defult'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:349 y:306
            OperatableStateMachine.add('suc',
                                        SetDIOState(io_service=self.io_service, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'check_suc', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

            # x:746 y:422
            OperatableStateMachine.add('succked',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: x[0]),
                                        transitions={'True': 'Move Arm To Obj Up Async', 'False': 'cnt++'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off},
                                        remapping={'input_value': 'pressure_sensor_val'})

            # x:361 y:213
            OperatableStateMachine.add('tool_select',
                                        DecisionState(outcomes=['pj', 'suction'], conditions=lambda x: x),
                                        transitions={'pj': 'Move Arm To Obj Pose Async', 'suction': 'Move Arm To Obj Pose Async 2'},
                                        autonomy={'pj': Autonomy.Off, 'suction': Autonomy.Off},
                                        remapping={'input_value': 'tool_name'})

            # x:351 y:100
            OperatableStateMachine.add('tool_select_defult',
                                        DecisionByParam(decided=self.tool_name, outcomes=['pj', 'suction']),
                                        transitions={'pj': 'Move Arm To Obj Pose Async', 'suction': 'Move Arm To Obj Pose Async 2'},
                                        autonomy={'pj': Autonomy.Off, 'suction': Autonomy.Off})

            # x:676 y:97
            OperatableStateMachine.add('Move Arm To Obj Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'planner': self.planner}),
                                        transitions={'finished': 'hiwin', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_zero', 'start_joints': 'expected_joints', 'velocity': 'velocity', 'exe_client': 'exe_client', 'ik_target_frame': 'ik_target_frame', 'target_joints': 'expected_joints'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
