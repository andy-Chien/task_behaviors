#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
import quaternion as qtn
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from tf import transformations as tf
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

'''
Created on 23.02.2022

@author: Andy Chien
'''

class MoveItPosePlanState(EventState):
	'''
	Uses MoveIt to plan the trajectory of target pose.

	-- robot_name        string  	The robot name for move group.
	-- velocity          int     	The velocity for robot motion.

	># pretarget_length  float   	The distance between target and pretarget.
	># pretarget_vector  float[] 	The vector for the diraction of pretarget position, given by [x, y, z].
	># start_joints 	 float[]		The joints of start pose, defult is current joints.
	># position			 float[]		The position of target pose in meters[x, y, z].
	># quaternion   	 float[]     The orientation of target pose representative by quaternion[w, x, y, z].

	#> joint_trajectory JointTrajectory  planned or executed trajectory

	<= done 						Target joint configuration has been planned.
	<= failed 				Failed to find a plan to the given joint configuration.
	'''


	def __init__(self, robot_name, velocity):
		'''
		Constructor
		'''
		super(MoveItPosePlanState, self).__init__(outcomes=['failed', 'done'],
										 input_keys=['pretarget_vector', 'pretarget_length', 'start_joints', 'position', 'quaternion'],
										 output_keys=['joint_trajectory'])
		self._group_name = robot_name
		self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
		self._move_group.set_planner_id("RRTConnectkConfigDefault")
		self._move_group.set_planning_time(1)
		self._velocity = velocity / 100.0 if 1 <= velocity <= 100 else 0.1
		self._result = None

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if len(self._result.joint_trajectory.points) > 0:
			userdata.joint_trajectory = self._result
			return 'done'
		else:
			return 'failed'

	def on_enter(self, userdata):
		self._result = None
		self._move_group.set_max_velocity_scaling_factor(self._velocity)
		self._move_group.set_max_acceleration_scaling_factor(self._velocity)
		joints_name = self._move_group.get_active_joints()
		if len(userdata.start_joints) == len(joints_name):
			start_state = self.generate_robot_state(joints_name, userdata.start_joints)
			self._move_group.set_start_state(start_state)
		else:
			self._move_group.set_start_state_to_current_state()
			
		p, q = userdata.position, userdata.quaternion
		if userdata.pretarget_length != 0:
			rot_mat = qtn.as_rotation_matrix(np.quaternion(q[0], q[1], q[2], q[3]))
			pretarget_offset = userdata.pretarget_length * np.matmul(rot_mat, np.array(userdata.pretarget_vector).T)
			pretarget_pos = pretarget_offset + np.array(userdata.position)
		else:
			pretarget_pos = None
		pose = Pose()
		if pretarget_pos is not None:
			pose.position = Point(pretarget_pos[0], pretarget_pos[1], pretarget_pos[2])
		else:
			pose.position = Point(p[0], p[1], p[2])
		pose.orientation = Quaternion(q[1], q[2], q[3], q[0])
		self._move_group.set_pose_target(pose)
		self._result = self._move_group.plan()
		if pretarget_pos is not None:
			pose.position = Point(p[0], p[1], p[2])
			jt = self._result.joint_trajectory
			start_state = self.generate_robot_state(jt.joint_names, jt.points[-1].positions)
			self._move_group.set_start_state(start_state)
			(cartesian_result, cartesian_fraction) = self._move_group.compute_cartesian_path([pose], 0.01, 0.0)
			jt.points += cartesian_result.joint_trajectory.points
			start_state = self.generate_robot_state(jt.joint_names, jt.points[0].positions)
			self._result = self._move_group.retime_trajectory(start_state, self._result, self._velocity, self._velocity, 
															  algorithm='iterative_time_parameterization')

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)

	def generate_robot_state(self, joints_name, start_joints):
		joint_state = JointState()
		joint_state.header = Header()
		joint_state.header.stamp = rospy.Time.now()
		joint_state.name = joints_name
		joint_state.position = start_joints
		state = RobotState()
		state.joint_state = joint_state
		return state
		