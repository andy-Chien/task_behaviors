#!/usr/bin/env python

import numpy as np
import quaternion as qtn
from flexbe_core import EventState

'''
Created on 25.02.2022

@author: Andy Chien
'''

class CoordinateTransformState(EventState):
	'''
	Get the grasp pose from GQCNN server.

	-- trans_position    float[]     The position of transformation in meter.
	-- trans_quaternion  float[]	 The quaternion of transfrom, given by [w, x, y, z]

	#> source_position   float[]     The position of source transform in meter.
	#> source_quaternion float[]     The quaternion of source transform, given by [w, x, y, z].

	># target_position   float[]     The position of target transform in meter.
	># target_quaternion float[]     The quaternion of target transform, given by [w, x, y, z].

	<= done 						 Transform done.
	'''

	def __init__(self, trans_position, trans_quaternion):
		'''
		Constructor
		'''
		super(CoordinateTransformState, self).__init__(outcomes=['done'],
											input_keys=['source_position', 'source_quaternion'],
											output_keys=['target_position', 'target_quaternion'])
		self._trans_position = trans_position
		self._trans_quaternion = trans_quaternion
		
	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		t_q, s_q = self._trans_quaternion, userdata.source_quaternion
		rot_mat = np.identity(4)
		rot_mat[:3, :3] = qtn.as_rotation_matrix(np.quaternion(t_q[0], t_q[1], t_q[2], t_q[3]))
		rot_mat[:3, 3] = np.array(self._trans_position)
		source_mat = np.identity(4)
		source_mat[:3, :3] = qtn.as_rotation_matrix(np.quaternion(s_q[0], s_q[1], s_q[2], s_q[3]))
		source_mat[:3, 3] = np.array(userdata.source_position)
		target_mat = np.matmul(rot_mat, source_mat)
		userdata.target_position = target_mat[:3, 3]
		userdata.target_quaternion = qtn.from_rotation_matrix(target_mat[:3, :3])
		return 'done'

	def on_enter(self, userdata):
		pass

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
