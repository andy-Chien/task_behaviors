import rclpy
import numpy as np
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, PoseStamped
from flexbe_core.proxy.proxy_publisher import ProxyPublisher
from flexbe_core import EventState
from open3d import io as o3d_io
import time

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
        super(MoveItAttachedObjState, self).__init__(outcomes=['done'],
                                                     input_keys=['link_name', 'pose'])
        # if len(namespace) > 0 and not namespace.startswith('/'):
        #     namespace = '/' + namespace 
        
        self.obj_name = obj_name
        self.mesh_file = mesh_file
        self.operation = operation
        self.obj_type = obj_type
        self.size = size
        self.pos = pos
        self.quat = quat

        self._logger = self._node.get_logger()
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

        self._logger.info('touch_links = {}'.format(self.touch_links))
        self._logger.info('mesh_file = {}, type = {}'.format(self.mesh_file, type(self.mesh_file)))
        print('print touch_links = {}'.format(touch_links))
        print('print mesh_file = {}'.format(mesh_file))

        if link_name.startswith('/'):
            self.link_name = link_name[1:]
        for i in range(len(touch_links)):
            if touch_links[i].startswith('/'):
                self.touch_links[i] = touch_links[i][1:]
        
        self._namespace = namespace
        
            


        ProxyPublisher._initialize(EventState._node)

        self._publisher = ProxyPublisher({self._attach_topic: AttachedCollisionObject})
        self._obj_publisher = ProxyPublisher({self._obj_topic: CollisionObject})

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        self._logger.info('touch_links = {}'.format(self.touch_links))
        self._logger.info('mesh_file = {}, type = {}'.format(self.mesh_file, type(self.mesh_file)))
        pose = Pose()

        up = userdata.pose
        ul = userdata.link_name
        if up == None:
            pose.position.x = self.pos[0]
            pose.position.y = self.pos[1]
            pose.position.z = self.pos[2]
            pose.orientation.w = self.quat[0]
            pose.orientation.x = self.quat[1]
            pose.orientation.y = self.quat[2]
            pose.orientation.z = self.quat[3]
        else:
            if isinstance(up, Pose):
                pose = up
            elif isinstance(up, PoseStamped):
                pose = up.pose
            elif isinstance(up, (list, np.ndarray)) and len(up) == 7:
                pose.position.x = up[0]
                pose.position.y = up[1]
                pose.position.z = up[2]
                pose.orientation.w = up[3]
                pose.orientation.x = up[4]
                pose.orientation.y = up[5]
                pose.orientation.z = up[6]
            else:
                self._logger.error('Input pose type: {} not supported = {}'.format(type(up)))
                return
            
        msg = AttachedCollisionObject()
        if ul == None:
            msg.link_name = self.link_name
            msg.object.header.frame_id = self.link_name
        else:
            utf = str(ul)
            if utf != '':
                if utf.startswith('/'):
                    msg.link_name = utf[1:]
                    msg.object.header.frame_id = utf[1:]
                elif self._namespace != '' and utf.split('_')[0] != self._namespace:
                    msg.link_name = self._namespace + '_' + utf
                    msg.object.header.frame_id = self._namespace + '_' + utf
                else:
                    msg.link_name = utf
                    msg.object.header.frame_id = utf

        msg.touch_links = self.touch_links
        msg.object.id = self.obj_name
        msg.object.pose = pose
        msg.object.operation = CollisionObject.ADD if 'add' in self.operation else CollisionObject.REMOVE

        ok = True
        if 'mesh' in self.obj_type and self.mesh_file != '':
            mesh_msg = Mesh()
            o3d_mesh = o3d_io.read_triangle_mesh(self.mesh_file)
            vertices = np.asarray(o3d_mesh.vertices)
            triangles = np.asarray(o3d_mesh.triangles)
            self._logger.error('o3d_mesh: {}'.format(o3d_mesh))

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
        elif self.obj_type != '': 
            sp = SolidPrimitive()
            sp.dimensions = self.size
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
        else:
            ok = False
        if ok:
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self._logger.error('msg: {}'.format(msg))
            self._publisher.publish(self._attach_topic, msg)
        if 'remove' in self.operation:
            time.sleep(0.1)
            self._obj_publisher.publish(self._obj_topic, msg.object)

        return 'done'