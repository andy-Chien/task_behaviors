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
from task_flexbe_states.hiwin_xeg32_gripper_client import HiwinXeg32GripperClient
from task_flexbe_states.moveit_canccel_execute_state import MoveitCancelExecuteState
from task_flexbe_states.set_DIO_state import SetDIOState
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
        self.add_parameter('io_service', '/ur_hardware_interface/set_io')
        self.add_parameter('sim', True)
        self.add_parameter('joint_names', dict())
        self.add_parameter('select_tool_by_input', False)
        self.add_parameter('namespace', '')
        self.add_parameter('group_name', 'ur_manipulator')

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        DecisionByParam.initialize_ros(node)
        DecisionState.initialize_ros(node)
        GetDIOState.initialize_ros(node)
        HiwinXeg32GripperClient.initialize_ros(node)
        MoveitCancelExecuteState.initialize_ros(node)
        SetDIOState.initialize_ros(node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async 2', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async', node)
        self.add_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async 2', node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        # x:1041 y:339, x:765 y:293
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['vacuum_io_pins', 'vacuum_io_vals', 'start_joints', 'pick_pose', 'pick_pose', 'pressure_sensor_pin', 'velocity'], output_keys=['target_joints'])
        _state_machine.userdata.vacuum_io_pins = [1]
        _state_machine.userdata.vacuum_io_vals = [0]
        _state_machine.userdata.exe_client = None
        _state_machine.userdata.translation_list = [.0, .0, 0.1]
        _state_machine.userdata.start_joints = None
        _state_machine.userdata.target_joints = None
        _state_machine.userdata.pick_pose = None
        _state_machine.userdata.tool_name = None
        _state_machine.userdata.translation_zero = [.0, .0, .0]
        _state_machine.userdata.pressure_sensor_pin = [2]
        _state_machine.userdata.velocity = 10

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:131 y:300
            OperatableStateMachine.add('Move Arm To Obj Up Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async',
                                            parameters={'wait': False}),
                                        transitions={'finished': 'select_tool_depend_on_input', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'target_joints': 'target_joints'})

            # x:521 y:215
            OperatableStateMachine.add('Move Arm To Obj Pose Async 2',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async 2',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'wait': False}),
                                        transitions={'finished': 'suc', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_zero', 'start_joints': 'start_joints', 'velocity': 'velocity', 'target_joints': 'target_joints'})

            # x:854 y:228
            OperatableStateMachine.add('Move Arm To Obj Up Pose Async 2',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Up Pose Async 2',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace, 'wait': False}),
                                        transitions={'finished': 'finished', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_list', 'start_joints': 'start_joints', 'velocity': 'velocity', 'target_joints': 'target_joints'})

            # x:907 y:409
            OperatableStateMachine.add('cancel_execution',
                                        MoveitCancelExecuteState(namespace=self.namespace, sim=False),
                                        transitions={'done': 'Move Arm To Obj Up Pose Async 2'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'exe_client': 'exe_client'})

            # x:551 y:419
            OperatableStateMachine.add('check_suc',
                                        GetDIOState(io_topic=self.io_topic, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'succked'},
                                        autonomy={'done': Autonomy.Off},
                                        remapping={'pins': 'pressure_sensor_pin', 'vals': 'pressure_sensor_val'})

            # x:854 y:99
            OperatableStateMachine.add('grasp',
                                        HiwinXeg32GripperClient(mode='close', direction=0, distance=0, speed=0, holding_stroke=0, holding_speed=0, holding_force=0, flag=0, namespace=self.namespace),
                                        transitions={'done': 'Move Arm To Obj Up Pose Async 2', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:124 y:163
            OperatableStateMachine.add('select_tool_depend_on_input',
                                        DecisionByParam(decided=self.select_tool_by_input, outcomes=['True', 'False']),
                                        transitions={'True': 'tool_select', 'False': 'tool_select_defult'},
                                        autonomy={'True': Autonomy.Off, 'False': Autonomy.Off})

            # x:344 y:413
            OperatableStateMachine.add('suc',
                                        SetDIOState(io_service=self.io_service, namespace=self.namespace, sim=self.sim),
                                        transitions={'done': 'check_suc', 'failed': 'failed'},
                                        autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'pins': 'vacuum_io_pins', 'vals': 'vacuum_io_vals'})

            # x:746 y:422
            OperatableStateMachine.add('succked',
                                        DecisionState(outcomes=['True', 'False'], conditions=lambda x: x[0]),
                                        transitions={'True': 'cancel_execution', 'False': 'check_suc'},
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

            # x:526 y:94
            OperatableStateMachine.add('Move Arm To Obj Pose Async',
                                        self.use_behavior(MoveArmToPoseAsyncSM, 'Move Arm To Obj Pose Async',
                                            parameters={'group_name': self.group_name, 'joint_names': self.joint_names, 'namespace': self.namespace}),
                                        transitions={'finished': 'grasp', 'failed': 'failed'},
                                        autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                        remapping={'target_pose': 'pick_pose', 'translation_list': 'translation_zero', 'start_joints': 'start_joints', 'velocity': 'velocity', 'target_joints': 'target_joints'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
