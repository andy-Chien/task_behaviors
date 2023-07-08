import rclpy
import numpy as np
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, PoseStamped
from flexbe_core.proxy.proxy_publisher import ProxyPublisher
from flexbe_core import EventState
from open3d import io as o3d_io

'''
Created on 24.02.2022

@author: Andy Chien
'''

class MoveItCollisionObjState(EventState):
    '''
    Use MoveIt to move robot by planned trajectory.

    -- obj_name
    -- mesh_file       string           move group name.
    -- operation          string           robot name or namespace.
    -- obj_type
    -- frame_id
    -- touch_links
    -- size

    <= done 						Robot move done.
    '''


    def __init__(self, mesh_file='', operation='add', obj_type='', frame_id='',
                 size=None, namespace='', obj_name='', pos=[0.0,0.0,0.0], quat=[1.0,0.0,0.0,0.0]):
        '''
        Constructor
        '''
        super(MoveItCollisionObjState, self).__init__(outcomes=['done'],
                                                      input_keys=['frame_id', 'pose'])
        # if len(namespace) > 0 and not namespace.startswith('/'):
        #     namespace = '/' + namespace 
        
        self.obj_name = obj_name
        self.mesh_file = mesh_file
        self.operation = operation
        self.obj_type = obj_type
        self.size = size
        self.pos = pos
        self.quat = quat

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._obj_topic = '/' + namespace + '/collision_object'
            self.frame_id = namespace + '_' + frame_id
        else:
            self._obj_topic = '/collision_object'
            self.frame_id = frame_id

        if frame_id.startswith('/'):
            self.frame_id = frame_id[1:]

        self._namespace = namespace
        
            


        ProxyPublisher._initialize(EventState._node)
        self._logger = self._node.get_logger()

        self._obj_publisher = ProxyPublisher({self._obj_topic: CollisionObject})

    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''
        self._logger.info('mesh_file = {}'.format(self.mesh_file))
        pose = Pose()

        up = userdata.pose
        ul = userdata.frame_id
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
            
        object_msg = CollisionObject()
        if ul == None:
            object_msg.header.frame_id = self.frame_id
        else:
            utf = str(ul)
            if utf != '':
                if utf.startswith('/'):
                    object_msg.header.frame_id = utf[1:]
                elif self._namespace != '' and utf.split('_')[0] != self._namespace:
                    object_msg.header.frame_id = self._namespace + '_' + utf
                else:
                    object_msg.header.frame_id = utf

        object_msg.id = self.obj_name
        object_msg.pose = pose
        object_msg.operation = CollisionObject.ADD if 'add' in self.operation else CollisionObject.REMOVE

        ok = True
        if 'mesh' in self.obj_type and self.mesh_file != '':
            mesh_msg = Mesh()
            o3d_mesh = o3d_io.read_triangle_mesh(self.mesh_file)
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
            object_msg.meshes.append(mesh_msg)
            object_msg.mesh_poses.append(Pose())
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
                object_msg.primitives.append(sp)
                object_msg.primitive_poses.append(Pose())
        else:
            ok = False
        if ok:
            self._obj_publisher.publish(self._obj_topic, object_msg)

        self._logger.info('object_msg.operation = {}'.format(object_msg.operation))

        return 'done'