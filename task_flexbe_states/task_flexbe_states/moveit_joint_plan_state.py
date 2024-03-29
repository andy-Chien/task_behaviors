#!/usr/bin/env python3

# import moveit_commander
import rclpy
import numpy as np
from moveit_msgs.msg import MoveItErrorCodes, RobotState, Constraints, JointConstraint
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState


'''
Created on 23.02.2022

@author: Andy Chien
'''

class MoveItJointsPlanState(EventState):
    '''
    Uses MoveIt to plan the trajectory of specified joints value.

    -- group_name       string                  move group name.
    -- joint_names      string[]                joints name of robot
    -- namespace        string                  robot name or namespace.
    -- planner          string                  planner name.

    -- velocity         int     	The velocity for robot motion, val within 0-100.
    -- time_out         float       time out for the planner.


    ># start_joints 	float[]		The joints of start pose, defult is current joints.
    ># target_joints	float[]		Target configuration of the joints.
                                    Same order as their corresponding names in joint_names.

    #> joint_trajectory JointTrajectory  planned or executed trajectory

    <= done 						Target joint configuration has been planned.
    <= failed 						Failed to find a plan to the given joint configuration.
    '''


    def __init__(self, group_name, joint_names, retry_cnt=3, namespace='', 
        planner='RRTConnectkConfigDefault', time_out=0.5, attempts=10):
        '''
        Constructor
        '''
        super(MoveItJointsPlanState, self).__init__(outcomes=['failed', 'done', 'retriable'],
            input_keys=['start_joints', 'target_joints', 'velocity'],
            output_keys=['joint_trajectory', 'planning_time', 'planning_error_code'])
        # group_name = ""
        self._group_name = group_name
        self._state_accepted = False
        # self._move_group = moveit_commander.MoveGroupCommander(self._group_name)
        # self._move_group.set_planner_id("RRTConnectkConfigDefault")
        # self._move_group.set_planning_time(1)
        # self._velocity = velocity / 100.0 if 1 <= velocity <= 100 else 0.1
        self._node = MoveItJointsPlanState._node
        self._logger = self._node.get_logger()
        self._retry_cnt = retry_cnt
        self._now_trying = 0

        if len(namespace) > 1 or (len(namespace) == 1 and not namespace.startswith('/')):
            namespace = namespace[1:] if namespace.startswith('/') else namespace
            self._action = '/' + namespace + '/move_action'
            self._joint_names = \
                [namespace + '_' + jn for jn in joint_names]
        else:
            self._action = '/move_action'
            self._joint_names = joint_names

        ProxyActionClient._initialize(EventState._node)
        self._client = ProxyActionClient({self._action: MoveGroup})

        self._goal = MoveGroup.Goal()
        self._goal.planning_options.plan_only = True
        self._goal.request.group_name = group_name
        self._goal.request.allowed_planning_time = time_out
        self._goal.request.planner_id = planner
        self._goal.request.num_planning_attempts = attempts
        cs = Constraints()
        for jn in self._joint_names:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            cs.joint_constraints.append(jc)
        self._goal.request.goal_constraints.append(cs)


    def execute(self, userdata):
        '''
        Execute this state
        '''
        if not self._state_accepted:
            return 'failed'

        # Check if the action has been finished
        if not self._client.has_result(self._action):
            return
        self._now_trying += 1
        result = self._client.get_result(self._action)
        userdata.planning_time = result.planning_time
        userdata.planning_error_code = result.error_code.val
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            userdata.joint_trajectory = result.planned_trajectory
            self._now_trying = 0
            return 'done'
        elif result.error_code.val == MoveItErrorCodes.INVALID_MOTION_PLAN and \
            self._now_trying < self._retry_cnt:
            self._now_trying += 1
            return 'retriable'
        else:
            self._logger.warn("Joint planning failed, error is {}.".format(result.error_code))
            self._now_trying = 0
            return 'failed'

    def on_enter(self, userdata):
        sj = np.array(userdata.start_joints, dtype=float)
        tj = np.array(userdata.target_joints, dtype=float)
        if np.any(np.absolute(sj) > np.pi * 2):
            sj = sj * np.pi / 180
        if np.any(np.absolute(tj) > np.pi * 2):
            tj = tj * np.pi / 180

        if not (len(sj) == len(tj) == len(self._joint_names)):
            self._logger.error(
                "Lengths of tj, sj and joint names are not equal, {}, {}, {}".format(
                    sj, tj, self._joint_names))
            self._state_accepted = False
            return

        # self._move_group.set_start_state(start_state)
        v = userdata.velocity
        v = v if 0 < v <= 1 else v / 100.0 if 1 <= v <= 100 else 0.1

        self._goal.request.start_state = self.generate_robot_state(self._joint_names, sj)
        self._goal.request.max_velocity_scaling_factor = v
        self._goal.request.max_acceleration_scaling_factor = v
        for jc, pos in zip(self._goal.request.goal_constraints[0].joint_constraints, tj):
            jc.position = pos

        try:
            self._client.send_goal(self._action, self._goal)
        except Exception as e:
            self._state_accepted = False
            Logger.logwarn('Failed to send the move group goal')

        self._state_accepted = True

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)

    def generate_robot_state(self, joint_names, start_joints):
        joint_state = JointState()
        joint_state.header.stamp = self._node.get_clock().now().to_msg()
        joint_state.name = joint_names
        joint_state.position = list(start_joints)
        state = RobotState()
        state.joint_state = joint_state
        return state