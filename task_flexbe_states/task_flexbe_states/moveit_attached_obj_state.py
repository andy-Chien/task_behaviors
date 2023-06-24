#!/usr/bin/env python3
import rclpy
import open3d as o3d
from moveit_msgs.msg import MoveItErrorCodes, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose
from flexbe_core.proxy.proxy_publisher import ProxyPublisher
from flexbe_core import EventState

'''
Created on 24.02.2022

@author: Andy Chien
'''

class MoveItAttachedObjState(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- obj_name
    -- mesh_file       string           move group name.
    -- operation          string           robot name or namespace.
    -- obj_type
    -- link_name
    -- touch_links
    -- size

    <= done 						Robot move done.
    '''


    def __init__(self, mesh_file='', operation='add', obj_type='', link_name='', \
                 touch_links=[], size=None, namespace='', obj_name='', pos=[0,0,0], quat=[1,0,0,0]):
        '''
        Constructor
        '''
        super(MoveItAttachedObjState, self).__init__(outcomes=['done'])
        if len(namespace) > 0 and not namespace.startswith('/'): 
            namespace = '/' + namespace 

        self._attach_topic = namespace + '/attached_collision_object'
        self.obj_name = obj_name
        self.mesh_file = mesh_file
        self.operation = operation
        self.type = obj_type
        self.link_name = link_name
        self.touch_links = touch_links
        self.size = size
        self.pos = pos
        self.quat = quat

        ProxyPublisher._initialize(EventState._node)
        self._logger = self._node.get_logger()

        self._publisher = ProxyPublisher({self._attach_topic: AttachedCollisionObject})

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        msg = AttachedCollisionObject()
        msg.link_name = self.link_name
        msg.touch_links = self.touch_links
        msg.object.id = self.obj_name
        msg.object.header
        msg.object.pose
        msg.object.operation = self.operation
        p = Pose()
        p.position.x = self.pos[0]
        p.position.x = self.pos[1]
        p.position.x = self.pos[2]
        p.orientation.w = self.quat[0]
        p.orientation.x = self.quat[1]
        p.orientation.y = self.quat[2]
        p.orientation.z = self.quat[3]

        if 'mesh' in self.obj_type and self.mesh_file != '':
            msg.object.meshes = 
            msg.object.mesh_poses.append(p)
        else: 
            sp = SolidPrimitive()
            sp.dimensions = self.size
            ok = True
            if 'box' in self.obj_type:
                sp.type = SolidPrimitive.BOX
            elif 'sphere' in self.obj_type:
                sp.type = SolidPrimitive.SPHERE
            elif 'cylinder' in self.obj_type:
                sp.type = SolidPrimitive.CYLINDER
            elif 'cone' in self.obj_type:
                sp.type = SolidPrimitive.CONE
            else:
                ok = False
            if ok:
                msg.object.primitives.append(sp)
                msg.object.primitive_poses.append(p)
        return 'done'

    def on_enter(self, userdata):
        pass

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)