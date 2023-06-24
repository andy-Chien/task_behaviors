import rclpy
import numpy as np
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from flexbe_core.proxy.proxy_publisher import ProxyPublisher
from flexbe_core import EventState
import open3d as o3d

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


    def __init__(self, mesh_file='', operation='add', obj_type='', link_name='',
                 touch_links=[], size=None, namespace='', obj_name='', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]):
        '''
        Constructor
        '''
        super(MoveItAttachedObjState, self).__init__(outcomes=['done'])
        if len(namespace) > 0 and not namespace.startswith('/'):
            namespace = '/' + namespace 
        
        self.obj_name = obj_name
        self.mesh_file = mesh_file
        self.operation = operation
        self.obj_type = obj_type
        self.size = size
        self.pos = pos
        self.quat = quat

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._attach_topic = '/' + namespace + '/attached_collision_object'
            self._obj_topic = '/' + namespace + '/collision_object'
            self.touch_links = [namespace + '_' + jn for jn in touch_links]
            self.link_name = namespace + '_' + link_name
        else:
            self._attach_topic = '/attached_collision_object'
            self._obj_topic = '/collision_object'
            self.link_name = link_name
            self.touch_links = touch_links
            


        ProxyPublisher._initialize(EventState._node)
        self._logger = self._node.get_logger()

        self._publisher = ProxyPublisher({self._attach_topic: AttachedCollisionObject})
        self._obj_publisher = ProxyPublisher({self._obj_topic: CollisionObject})

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        self._logger.info('touch_links = {}'.format(self.touch_links))
        self._logger.info('mesh_file = {}'.format(self.mesh_file))
        pose = Pose()
        pose.position.x = self.pos[0]
        pose.position.x = self.pos[1]
        pose.position.x = self.pos[2]
        pose.orientation.w = self.quat[0]
        pose.orientation.x = self.quat[1]
        pose.orientation.y = self.quat[2]
        pose.orientation.z = self.quat[3]
        msg = AttachedCollisionObject()
        msg.link_name = self.link_name
        msg.touch_links = self.touch_links
        msg.object.id = self.obj_name
        msg.object.header.frame_id = self.link_name
        msg.object.pose = pose
        msg.object.operation = CollisionObject.ADD if 'add' in self.operation else CollisionObject.REMOVE

        if 'mesh' in self.obj_type and self.mesh_file != '':
            mesh_msg = Mesh()
            o3d_mesh = o3d.io.read_triangle_mesh(self.mesh_file)
            vertices = np.asarray(o3d_mesh.vertices)
            triangles = np.asarray(o3d_mesh.triangles)
            for v in vertices:
                p = Point()
                p.x, p.y, p.z = v
                mesh_msg.vertices.append(p)
            for t in triangles:
                mt = MeshTriangle()
                mt.vertex_indices = np.array(t, dtype=np.uint32)
                mesh_msg.triangles.append(mt)
            msg.object.meshes.append(mesh_msg)
            msg.object.mesh_poses.append(Pose())
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
                msg.object.primitive_poses.append(Pose())
        self._publisher.publish(self._attach_topic, msg)
        if 'remove' in self.operation:
            self._obj_publisher.publish(self._obj_topic, msg.object)

        return 'done'

    def on_enter(self, userdata):
        pass

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)