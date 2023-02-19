#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_msgs.msg import MoveItErrorCodes, ExecuteTrajectoryAction, RobotTrajectory, ExecuteTrajectoryGoal
from flexbe_core.proxy.proxy_action_client import ProxyActionClient
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
		self._exe_topic = '/execute_trajectory'
		self._exe_client = ProxyActionClient({self._exe_topic: ExecuteTrajectoryAction})
		self._running = False

	def stop(self):
		pass

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._exe_client.has_result(self._exe_topic):
			result = self._exe_client.get_result(self._exe_topic)
			error_code = result.error_code.val
			if error_code == MoveItErrorCodes.SUCCESS or error_code == MoveItErrorCodes.PREEMPTED:
				self._running = False
				return 'done'
			elif error_code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
				rospy.logwarn('[MoveIt Execute Trajectory State]: ' + str(error_code))
				self._running = False
				return 'collision'
			else:
				rospy.logerr('[MoveIt Execute Trajectory State]: MoveItErrorCodes = {}'.format(error_code))
				self._running = False
				return 'failed'
		else:
			self._running = True
			return 'running'

	def on_enter(self, userdata):
		if not self._running:
			goal = ExecuteTrajectoryGoal(userdata.joint_trajectory)
			self._exe_client.send_goal(self._exe_topic, goal)

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)