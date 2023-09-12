'''
Created on 26/02/2022

@author: Andy Chien
'''
from task_msgs.srv import PackingPlanning
from flexbe_core import EventState, Logger
from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_transform_listener import ProxyTransformListener
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import quaternion as qtn
import numpy as np
import ros2_numpy
from tf2_ros import TransformException
import rclpy
import time


class PackingPlanClient(EventState):
    '''
    Call packing planning server

    -- namespace   string   Namespace of service

    ># box_cloud   oped3d_poind_cloud   A point cloud of packing box
    ># obj_cloud   oped3d_poind_cloud   A point cloud of packing obj

    #> packing_pose geometry_msgs/Pose  Pose to place the obj

    <= done        Packing plan finished.
    <= failed      Packing plan failed.
    '''

    def __init__(self, namespace='', frame_id='base_link'):
        '''
        Constructor
        '''
        super(PackingPlanClient, self).__init__(outcomes=['done', 'failed'],
                                          input_keys=['box_cloud', 'obj_cloud', 'box_size', 
                                                      'is_first_obj', 'init_pose', 'camera_trans_box', 'frame'],
                                          output_keys=['packing_pose'])
        self._service_name = '/packing_planning'

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._service_name = '/' + namespace + self._service_name
            self._frame_id = namespace + '_' + frame_id
        else:
            self._service_name = namespace + self._service_name
            self._frame_id = namespace + frame_id

        self._service = ProxyServiceCaller({self._service_name: PackingPlanning})
        ProxyTransformListener._initialize(self._node)
        self.tf_listener = ProxyTransformListener().listener()
        self.tf_buffer = self.tf_listener.buffer
        
    def execute(self, userdata):
        if not self._service.done(self._service_name):
            return
        result = self._service.result(self._service_name)
        rpp = result.relative_place_pose
        Logger.logwarn('relative_place_pose = {}'.format(rpp))

        # xyz_vec = np.identity(3)
        # xyz_vec[0][0] = rpp.angular.x
        # xyz_vec[1][1] = rpp.angular.y
        # xyz_vec[2][2] = rpp.angular.z
        xyz_qtn = qtn.from_rotation_vector(np.array([0.0, 0.0, rpp.angular.z]))
        box_m_place = np.mat(np.identity(4))
        box_m_place[:3, 3] = [[rpp.linear.x], [-1*rpp.linear.y], [rpp.linear.z]]
        # box_m_place[:3, 3] = [[rpp.linear.x], [rpp.linear.y], [0.1]]
        box_m_place[:3, :3] = qtn.as_rotation_matrix(xyz_qtn)[:3, :3]
        
        box_m_place_ori = np.mat(np.identity(4))
        box_m_place_pos = np.mat(np.identity(4))
        box_m_place_ori[:3, :3] = box_m_place[:3, :3]
        box_m_place_pos[:3, 3] = box_m_place[:3, 3]

        
        try:
            tf_transform = self.tf_buffer.lookup_transform(
                self._frame_id,
                userdata.frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            Logger.logwarn('lookupTransform for tip failed!')
            return
        
        transform = np.identity(4)
        transform[:3, :3] = qtn.as_rotation_matrix(
            np.quaternion(tf_transform.transform.rotation.w, 
                          tf_transform.transform.rotation.x, 
                          tf_transform.transform.rotation.y, 
                          tf_transform.transform.rotation.z)
        )
        transform[:3, 3] = np.array([tf_transform.transform.translation.x, 
                                     tf_transform.transform.translation.y, 
                                     tf_transform.transform.translation.z])
        
        base_2_box = transform * np.mat(userdata.camera_trans_box)
        
        ip = userdata.init_pose
        base_m_init = np.mat(np.identity(4))
        base_m_init[:3, 3] = base_2_box[:3, 3]
        base_m_init[:3, :3] = qtn.as_rotation_matrix(
            qtn.from_float_array(ip[3:]))[:3, :3]
        
        base_m_init_ori = np.mat(np.identity(4))
        base_m_init_pos = np.mat(np.identity(4))
        base_m_init_ori[:3, :3] = base_m_init[:3, :3]
        base_m_init_pos[:3, 3] = base_m_init[:3, 3]
        
        base_m_place = np.mat(np.identity(4))
        base_m_place_ori = box_m_place_ori * base_m_init_ori
        base_m_place_pos = base_m_init_pos * box_m_place_pos
        base_m_place[:3, :3] = base_m_place_ori[:3, :3]
        base_m_place[:3, 3] = base_m_place_pos[:3, 3]
        ps = PoseStamped()
        
        ps.header.frame_id = self._frame_id
        p = ps.pose
        po = p.orientation
        p.position.x = base_m_place[0, 3]
        p.position.y = base_m_place[1, 3]
        p.position.z = base_m_place[2, 3]
        [po.w, po.x, po.y, po.z] = qtn.as_float_array(
            qtn.from_rotation_matrix(base_m_place[:3, :3]))

        Logger.logwarn('Place pose = {}'.format(p))
        userdata.packing_pose = p
        return 'done' if result.success else 'failed'

    def on_enter(self, userdata):
        req = PackingPlanning.Request()
        req.box_cloud = self.cloud_msg_from_open3d(userdata.box_cloud)
        req.obj_cloud = self.cloud_msg_from_open3d(userdata.obj_cloud)
        req.box_size = userdata.box_size
        req.is_first_obj = userdata.is_first_obj
        self._service.call_async(self._service_name, req)

    def cloud_msg_from_open3d(self, o3d_pcd):
        pcd = np.asarray(o3d_pcd.points)
        # data = np.zeros(pcd.shape[0], dtype=[
        #     ('x', np.float32),
        #     ('y', np.float32),
        #     ('z', np.float32)
        # ])
        data = dict()
        data['xyz'] = pcd
        msg = ros2_numpy.msgify(PointCloud2, data)
        Logger.logwarn('pcd.shape = {}, height = {}, width = {}, point_step = {}, row_step = {}, data.len = {}'.format(
            pcd.shape, msg.height, msg.width, msg.point_step, msg.row_step, len(msg.data)))
        return msg