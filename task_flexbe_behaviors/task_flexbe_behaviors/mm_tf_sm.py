#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from task_flexbe_states.calculate_tf_state import CalculateTfState
from task_flexbe_states.get_single_armarker_state import GetSingleArmarkerState
from task_flexbe_states.get_tf_state import GetTfState
from task_flexbe_states.set_static_tf_state import SetStaticTfState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thur Jun 22 2023
@author: TaiTing Tsai
'''
class MMtfSM(Behavior):
	'''
	calibrate moblie manipulator tf_static
	'''


	def __init__(self, node):
		super(MMtfSM, self).__init__()
		self.name = 'MM tf'

		# parameters of this behavior
		self.add_parameter('namespace', '')
		self.add_parameter('group_name', 'ur_manipulator')
		self.add_parameter('joint_names', dict())
		self.add_parameter('random_areas', dict())
		self.add_parameter('planner_id', 'RRTConnectkConfigDefault')
		self.add_parameter('terminal_rounds', 100)
		self.add_parameter('do_evaluation', False)
		self.add_parameter('using_areas', dict())
		self.add_parameter('eval_rounds', 100)

		# references to used behaviors
		OperatableStateMachine.initialize_ros(node)
		ConcurrencyContainer.initialize_ros(node)
		PriorityContainer.initialize_ros(node)
		Logger.initialize(node)
		CalculateTfState.initialize_ros(node)
		GetSingleArmarkerState.initialize_ros(node)
		GetTfState.initialize_ros(node)
		SetStaticTfState.initialize_ros(node)

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:675 y:527, x:183 y:548
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['velocity', 'planner'])
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.exe_client = None
		_state_machine.userdata.curr_area = 0
		_state_machine.userdata.running_cnt = 0
		_state_machine.userdata.goal_pos = [-0.5571767752784397, 0.17723418155, 0.10149310679963709]
		_state_machine.userdata.goal_rot = [0.41909769121020485, 0.8500430253557405, -0.1266676839251758, -0.2928127014242511]
		_state_machine.userdata.planner = "BiTRRT"
		_state_machine.userdata.transform_list = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


		with _state_machine:
			# x:588 y:154
			OperatableStateMachine.add('get_cali_armarker',
										GetSingleArmarkerState(dictionary_id_name='DICT_5X5_250', marker_size=0.053, camera_info_topic='/camera/color/camera_info', image_topic='/camera/color/image_raw'),
										transitions={'done': 'calculate_tf', 'failed': 'get_cali_armarker'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'armarker_pos': 'transform'})

			# x:585 y:304
			OperatableStateMachine.add('get_tf_trans',
										GetTfState(parent_frame=['world', 'camera_color_frame'], child_frame=['world_marker', 'base_link']),
										transitions={'done': 'calculate_tf', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'transform_list': 'transform_list'})

			# x:781 y:408
			OperatableStateMachine.add('update_tf',
										SetStaticTfState(parent_frame='world', child_frame='base_link'),
										transitions={'done': 'calculate_tf'},
										autonomy={'done': Autonomy.Off},
										remapping={'transform_mat': 'updated_transform'})

			# x:816 y:106
			OperatableStateMachine.add('calculate_tf',
										CalculateTfState(tf_right_muti_count=1, tf_left_muti_count=1),
										transitions={'done': 'update_tf', 'failed': 'calculate_tf', 'listen_tf': 'get_tf_trans'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off, 'listen_tf': Autonomy.Off},
										remapping={'transform': 'transform', 'listened_transform': 'transform_list', 'updated_transform': 'updated_transform'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
