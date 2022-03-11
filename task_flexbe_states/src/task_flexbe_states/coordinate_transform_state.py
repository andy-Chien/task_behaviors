#!/usr/bin/env python3

import numpy as np
import quaternion as qtn
from flexbe_core import EventState

'''
Created on 25.02.2022

@author: Andy Chien
'''

class CoordinateTransformState(EventState):
	'''
	Trans source pose to target pose by trans matrix.

	#> trans_position    float[]     The position of transformation in meter.
	#> trans_quaternion  float[]	 The quaternion of transfrom, given by [w, x, y, z]
	#> source_position   float[]     The position of source transform in meter.
	#> source_quaternion float[]     The quaternion of source transform, given by [w, x, y, z].
	#> obj_quat_tool     float[]	 The quaternion between object and tool coordinate, given by [w, x, y, z].

	># target_position   float[]     The position of target transform in meter.
	># target_quaternion float[]     The quaternion of target transform, given by [w, x, y, z].

	<= done 						 Transform done.
	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(CoordinateTransformState, self).__init__(outcomes=['done'],
											input_keys=['trans_position', 'trans_quaternion','source_position', 'source_quaternion', 'obj_quat_tool'],
											output_keys=['target_position', 'target_quaternion'])
		
	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		t_q, s_q, o_q_t = userdata.trans_quaternion, userdata.source_quaternion, userdata.obj_quat_tool 
		rot_mat = np.identity(4)
		rot_mat[:3, :3] = qtn.as_rotation_matrix(np.quaternion(t_q[0], t_q[1], t_q[2], t_q[3]))
		rot_mat[:3, 3] = np.array(userdata.trans_position)
		source_mat = np.identity(4)
		source_mat[:3, :3] = qtn.as_rotation_matrix(np.quaternion(s_q[0], s_q[1], s_q[2], s_q[3]))
		source_mat[:3, 3] = np.array(userdata.source_position)
		o_mat_o = np.identity(4)
		o_mat_o[:3, :3] = qtn.as_rotation_matrix(np.quaternion(o_q_t[0], o_q_t[1], o_q_t[2], o_q_t[3]))
		source_mat = np.matmul(source_mat, o_mat_o)
		target_mat = np.matmul(rot_mat, source_mat)
		# target_mat = np.matmul(rot_mat, source_mat)
		userdata.target_position = target_mat[:3, 3]
		userdata.target_quaternion = qtn.as_float_array(qtn.from_rotation_matrix(target_mat[:3, :3]))
		print('target_position = {}'.format(userdata.target_position))
		print('target_quaternion = {}'.format(userdata.target_quaternion))
		return 'done'

	def on_enter(self, userdata):
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
