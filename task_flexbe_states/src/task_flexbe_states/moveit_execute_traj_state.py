#!/usr/bin/env python3

import rospy
import threading
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes
from flexbe_core import EventState

'''
Created on 24.02.2022

@author: Andy Chien
'''

class MoveItExecuteTrajectoryState(EventState):
	'''
	Use MoveIt to move robot by planned trajectory.

	-- robot_name         string           move group name

	># joint_trajectory   JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	<= collision 				    Robot during collision.
	'''


	def __init__(self, robot_name):
		'''
		Constructor
		'''
		super(MoveItExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed', 'collision', 'running'],
											input_keys=['joint_trajectory', 'block_execute'])
		self._group_name = robot_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE
		self._thread = None

	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._thread is not None and self._thread.is_alive():
			return 'running'

		if self._result == MoveItErrorCodes.SUCCESS or self._result == MoveItErrorCodes.PREEMPTED:
			self._thread = None
			return 'done'
		elif self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			self._thread = None
			rospy.logwarn('[MoveIt Execute Trajectory State]: ' + str(self._result))
			return 'collision'
		else:
			self._thread = None
			rospy.logerr('[MoveIt Execute Trajectory State]: MoveItErrorCodes = {}, typt = {}'.format(self._result, type(self._result)))
			return 'failed'

	def on_enter(self, userdata):
		if userdata.block_execute:
			print('1')
			self._result = self._move_group.execute(userdata.joint_trajectory)
		else:
			if self._thread is None:
				print('2')
				self._thread = threading.Thread(target = self.execute_trajectory, 
												args=(userdata.joint_trajectory,))
				self._thread.start()

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)

	def execute_trajectory(self, jt):
		self._result = self._move_group.execute(jt)