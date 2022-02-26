#!/usr/bin/env python

import rospy
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from gqcnn.srv import GQCNNGraspPlanner

'''
Created on 24.02.2022

@author: Andy Chien
'''

class GQCNNGraspPlanState(EventState):
	'''
	Get the grasp pose from GQCNN server.

	-- grasp_service    string      The Name of grasp planning service.

	># grasp_posision   float[]     The position of grasp pose.
	># grasp_quaternion float[]     The quaternion of grasp pose.

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	'''

	def __init__(self, grasp_service):
		'''
		Constructor
		'''
		super(GQCNNGraspPlanState, self).__init__(outcomes=['done', 'failed'],
											output_keys=['grasp_posision', 'grasp_quaternion'])
		self._grasp_service = grasp_service
		self._gqcnn_client = ProxyServiceCaller({self._grasp_service: GQCNNGraspPlanner})
		self._result = None
		
	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self._result is not False:
			p, q = self._result.grasp.pose.position, self._result.grasp.pose.orientation
			userdata.grasp_position = [p.x, p.y, p.z]
			userdata.grasp_quaternion = [q.w, q.x, q.y, q.z]
			return 'done'
		else:
			rospy.logerr('[GQCNN Grasp Plan State]: Grasp plan failed')
			return 'failed'

	def on_enter(self, userdata):
		self._result = self._gqcnn_client.call(self._grasp_service, {})

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
