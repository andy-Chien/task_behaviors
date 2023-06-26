#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from gqcnn_interfaces.srv import GQCNNGraspPlanner
from sensor_msgs.msg import CameraInfo, Image
import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
import numpy as np
import quaternion as qtn
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
'''
Created on 21.06.2023

@author: TaiTing Tsai
'''

class GetObjPointCloudState(EventState):
    '''
    Get the grasp pose from GQCNN server.

    -- grasp_service    string      The Name of grasp planning service.

    ># grasp_posision   float[]     The position of grasp pose.
    ># grasp_quaternion float[]     The quaternion of grasp pose.

    <= done 						Robot move done.
    <= failed 						Robot move failed.
    '''

    def __init__(self, depth_camera_info, depth_image_topic, tip_link, camera_frame):
        '''
        Constructor
        '''
        super(GetObjPointCloudState, self).__init__(outcomes=['done', 'failed', 'second', 'third'],
                                                    output_keys=['obj_pointcloud'])
        
        self._node = GetObjPointCloudState._node
        ProxySubscriberCached._initialize(self._node)
        self._logger = self._node.get_logger().info
        self.tip_link = tip_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)
        
        self._depth_camera_info = depth_camera_info
        self._depth_image_topic = depth_image_topic
        
        self._depth_camera_info_subscriber_ = ProxySubscriberCached({self._depth_camera_info : CameraInfo})
        self._depth_image_subscriber_ = ProxySubscriberCached({self._depth_image_topic : Image})
        self.info_received = False
        self.color_image_received = False
        self.depth_image_received = False
        self.camera_frame = camera_frame
        self.bridge = CvBridge()

        self.depth_camera_info = None
        self.depth_img = None
        self.depthpcd = []
        self.cnt_depth_img = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if not self.info_received or not self.depth_image_received and self.cnt_depth_img != 3:
            self.cnt_depth_img = 0
            return 'failed'
        
        if self.cnt_depth_img == 1:
            self.info_received = False
            self.depth_image_received = False
            return 'second'
        
        elif self.cnt_depth_img == 2:
            self.info_received = False
            self.depth_image_received = False
            return 'third'
        
        else:
            self.cnt_depth_img = 0
            self.info_received = False
            self.depth_image_received = False

            merged_points = np.concatenate([np.asarray(cloud.points) for cloud in self.depthpcd])
            o3d_points = o3d.cuda.pybind.utility.Vector3dVector(merged_points)
            merged_cloud = o3d.geometry.PointCloud(o3d_points)
            o3d.visualization.draw_geometries([merged_cloud], window_name='Open3D', width=3840, height=2160, \
            left=50, top=50, point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False)
            userdata.obj_pointcloud = merged_cloud

            return 'done'


    def on_enter(self, userdata):
        if self._depth_camera_info_subscriber_.has_msg(self._depth_camera_info):
            self.get_depth_camera_Info(self._depth_camera_info_subscriber_.get_last_msg(self._depth_camera_info))
        if self._depth_image_subscriber_.has_msg(self._depth_image_topic):
            self.get_depth_image_Info(self._depth_image_subscriber_.get_last_msg(self._depth_image_topic))



        if self.info_received and self.depth_image_received and self.cnt_depth_img != 3:
            self.cnt_depth_img += 1
            cam_intrinsic = o3d.camera.PinholeCameraIntrinsic(int(self.depth_camera_info.width), 
                                                                int(self.depth_camera_info.height),
                                                                float(self.depth_camera_info.k[0]),
                                                                float(self.depth_camera_info.k[4]),
                                                                float(self.depth_camera_info.k[2]),
                                                                float(self.depth_camera_info.k[5]))
            cam_intrinsic.intrinsic_matrix = [[float(self.depth_camera_info.k[0]), 0, float(self.depth_camera_info.k[2])], 
                                                 [0, float(self.depth_camera_info.k[4]), float(self.depth_camera_info.k[5])], 
                                                [0, 0, 1]]
            cam = o3d.camera.PinholeCameraParameters()
            cam.intrinsic = cam_intrinsic
            
            try:
                camera_trans_tip = self.tf_buffer.lookup_transform(
                    'world',
                    self.tip_link,
                    rclpy.time.Time())
                world_trans_camera = self.tf_buffer.lookup_transform(
                    'world',
                    self.camera_frame,
                    rclpy.time.Time()
                )
            except TransformException as ex:
                Logger.logwarn('lookupTransform for tip failed!')
                return
            center = np.array([[camera_trans_tip.transform.translation.x], 
                                [camera_trans_tip.transform.translation.y], 
                                [camera_trans_tip.transform.translation.z]])
            # center = np.array([[0.05], 
            #                     [0.05], 
            #                     [0.46]])

            cam_2_tip = qtn.as_rotation_matrix(np.quaternion(camera_trans_tip.transform.rotation.w, 
                                                             camera_trans_tip.transform.rotation.x, 
                                                             camera_trans_tip.transform.rotation.y, 
                                                             camera_trans_tip.transform.rotation.z))
            world_2_camera = np.identity(4)
            world_2_camera[:3, :3] = qtn.as_rotation_matrix(
                np.quaternion(world_trans_camera.transform.rotation.w, 
                                world_trans_camera.transform.rotation.x, 
                                world_trans_camera.transform.rotation.y, 
                                world_trans_camera.transform.rotation.z)
            )
            world_2_camera[:3, 3] = np.array([world_trans_camera.transform.translation.x, 
                                                world_trans_camera.transform.translation.y, 
                                                world_trans_camera.transform.translation.z])
            
            rot_vec = cam_2_tip[:3, 2]
                        

            if self.cnt_depth_img == 1:
                R = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

            elif self.cnt_depth_img == 2:#rotate degree -120
                R = np.array([[-0.5, 0.8660254, 0.], [-0.8660254, -0.5, 0.], [0., 0., 1.]])

            elif self.cnt_depth_img == 3:#rotate degree -240

                R = np.array([[-0.5, -0.8660254, 0.], [0.8660254, -0.5, 0.], [0., 0., 1.]])
 


            # R = cam_2_tip
            depthpcd = o3d.open3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(self.depth_img.astype(np.float32) * 0.001), 
                                                                              cam.intrinsic, 
                                                                              depth_trunc=0)

            depthpcd = depthpcd.transform(world_2_camera)
            depthpcd = depthpcd.rotate(R, center)
            self.depthpcd.append(depthpcd)

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)

    def get_depth_camera_Info(self, data):
        self.depth_camera_info = data
        self.info_received = True

    def get_depth_image_Info(self, data):
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        depth_image = np.array([[x if x < 450 and 320<j<555 and 180<i<410 else np.nan for j, x in enumerate(row)] for i, row in enumerate(depth_image)])
        self.depth_img = depth_image
        self.depth_image_received = True
