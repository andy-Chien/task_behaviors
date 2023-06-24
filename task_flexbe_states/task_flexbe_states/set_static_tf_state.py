#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
import rclpy
import numpy as np
import quaternion as qtn
from geometry_msgs.msg import Pose, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy,HistoryPolicy,QoSDurabilityPolicy,QoSPolicyKind,LivelinessPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.duration import Duration
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
'''
Created on 23.06.2023

@author: TaiTing Tsai
'''

class SetStaticTfState(EventState):
    '''
    Set the updated static tf and publish it.

    -- parent_frame    string      The Name of tf parent frame.
    -- child_frame     string      The Name of tf child frame.

    ># transform_mat               Transformation matrix for tf calculation.

    <= done 					   Robot move done.
    '''

    def __init__(self, parent_frame, child_frame):
        '''
        Constructor
        '''
        super(SetStaticTfState, self).__init__(outcomes=['done'],
                                               input_keys=['transform_mat'])
        

        self._node = SetStaticTfState._node
        self._logger = self._node.get_logger().info
        ProxyPublisher._initialize(self._node)

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        qos = QoSProfile(depth=1, 
                         reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                         history= HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                         durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                         lifespan=Duration(nanoseconds=_rclpy.RMW_DURATION_INFINITE),
                         deadline=Duration(nanoseconds=_rclpy.RMW_DURATION_INFINITE),
                         liveliness=LivelinessPolicy.AUTOMATIC,
                         liveliness_lease_duration=Duration(nanoseconds=_rclpy.RMW_DURATION_INFINITE),
                         )

        self.tf_static_broadcaster = ProxyPublisher({'/tf_static':TFMessage},qos=qos)
        # self.tf_broadcaster = ProxyPublisher({'/tf':TFMessage})
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        rot_mat = np.delete(userdata.transform_mat, 3, axis=0)
        rot_mat = np.delete(userdata.transform_mat, 3, axis=1)
        quat = qtn.from_rotation_matrix(rot_mat)
        new_trans = TransformStamped()
        new_trans.header.stamp = self._node.get_clock().now().to_msg()
        new_trans.header.frame_id = self.parent_frame 
        new_trans.child_frame_id = self.child_frame 
        new_trans.transform.translation.x = float(userdata.transform_mat[0][3])
        new_trans.transform.translation.y = float(userdata.transform_mat[1][3])
        new_trans.transform.translation.z = float(userdata.transform_mat[2][3])
        new_trans.transform.rotation.w = quat.w
        new_trans.transform.rotation.x = quat.x
        new_trans.transform.rotation.y = quat.y
        new_trans.transform.rotation.z = quat.z
        
        new_tf = TFMessage()
        new_tf.transforms.append(new_trans)
        self.tf_static_broadcaster.publish('/tf_static', new_tf)

        return 'done'


    def on_enter(self, userdata):
        pass
    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)
