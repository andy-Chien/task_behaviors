#!/usr/bin/env python

import rospy
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
		super(MoveItExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed', 'collision'],
											input_keys=['joint_trajectory'])
		self._group_name = robot_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._result = MoveItErrorCodes.FAILURE

	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
	
		if self._result == MoveItErrorCodes.SUCCESS:
			return 'done'
		elif self._result == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
			rospy.logwarn('[MoveIt Execute Trajectory State]: ' + str(self._result))
			return 'collision'
		else:
			rospy.logerr('[MoveIt Execute Trajectory State]: MoveItErrorCodes = ' + str(self._result))
			return 'failed'

	def on_enter(self, userdata):
		self._result = self._move_group.execute(userdata.joint_trajectory)

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
