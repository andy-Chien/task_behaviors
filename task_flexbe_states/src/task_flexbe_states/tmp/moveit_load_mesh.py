#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
from flexbe_core import EventState
from tf_conversions import transformations as tf_trans

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import sys
import rospy
import moveit_commander
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('load_mesh_node')
robot = moveit_commander.RobotCommander()
psi = moveit_commander.PlanningSceneInterface()
mg = moveit_commander.MoveGroupCommander('manipulator')

print(mg.get_planning_frame())
print(robot.get_group_names())


# pose_goal = Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.4
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.4

# mg.set_pose_target(pose_goal)
# plan=mg.go(wait=True)
# mg.stop()
# mg.clear_pose_targets()



file = '/home/andy/packing_ws/src/objects_model/scene_objects/blue_bin.STL'
obj_name = 'blue_bin_1'
scale = 0.001
position = [1,1,0]
rotation = np.array([0,0,0]) * np.pi / 180
parent_frame = 'base_link'


q = tf_trans.quaternion_from_euler(rotation[0], rotation[1], rotation[2], 'sxyz')
mesh_pose = PoseStamped()
mesh_pose.header.frame_id = parent_frame
mesh_pose.pose.position = Point(position[0], position[1], position[2])
mesh_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
psi.add_mesh(obj_name, mesh_pose, file, (scale, scale, scale))
print(psi.get_known_object_names())
rospy.spin()