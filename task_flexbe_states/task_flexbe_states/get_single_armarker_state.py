#!/usr/bin/env python3
import os
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
import numpy as np
import math
from geometry_msgs.msg import Pose
import quaternion as qtn
'''
Created on 21.06.2023

@author: TaiTing Tsai
'''

class GetSingleArmarkerState(EventState):
    '''
    Get single aruco marker transformation pose

    -- dictionary_id_name    string      The aruco_dictionary_id.
    -- marker_size           float       The aruco marker size.
    -- camera_info_topic     string      Camera info topic.
    -- image_topic           string      Image topic.

    ># armarker_pos                      Transformation pose form camera to aruco marker.

    <= done 						     Get aruco marker done.
    <= failed                            Get aruco marker failed.
    '''

    def __init__(self, dictionary_id_name, marker_size, camera_info_topic, image_topic):
        '''
        Constructor
        '''
        super(GetSingleArmarkerState, self).__init__(outcomes=['done', 'failed'],
                                                    output_keys=['armarker_pos'])
        
        self._node = GetSingleArmarkerState._node
        ProxySubscriberCached._initialize(self._node)
        self._logger = self._node.get_logger().info
        
        self.marker_size = marker_size
        self.dictionary_id_name = dictionary_id_name
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.bridge = CvBridge()
        self.camera_frame = None
        
        self._camera_info_subscriber_ = ProxySubscriberCached({self.camera_info_topic : CameraInfo})
        self._image_subscriber_ = ProxySubscriberCached({self.image_topic : Image})
        self.info_received = False

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        #camera intrinsic
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.get_marker = False
        self.get_mask_image = False
        self.marker_ids =None
        self.armarker_pos = Pose()
        self.cnt = 0
        
    def stop(self):
        pass

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if not self.info_received or not self.get_marker:
            return 'failed'
        
        else:
            userdata.armarker_pos = self.armarker_pos
            self.marker_ids = None
            return 'done'


    def on_enter(self, userdata):
        if self._camera_info_subscriber_.has_msg(self.camera_info_topic):
            self.get_camera_Info(self._camera_info_subscriber_.get_last_msg(self.camera_info_topic))
        if self._image_subscriber_.has_msg(self.image_topic):
            self.get_image_Info(self._image_subscriber_.get_last_msg(self.image_topic))



        if self.info_received and self.get_marker and self.marker_ids != None:

            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)
            self.cv_image = cv2.aruco.drawAxis(self.cv_image, self.intrinsic_mat, self.distortion, rvecs[0], tvecs[0], 0.02)
            cv2.imshow('QueryImage', self.cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            for i in range(len(self.marker_ids)):
                self.armarker_pos = Pose()
                self.armarker_pos.position.x = tvecs[i][0][0]
                self.armarker_pos.position.y = tvecs[i][0][1]
                self.armarker_pos.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                
                quat = qtn.from_rotation_matrix(rot_matrix)

                self.armarker_pos.orientation.w = quat.w
                self.armarker_pos.orientation.x = quat.x
                self.armarker_pos.orientation.y = quat.y
                self.armarker_pos.orientation.z = quat.z
            self._logger('++++++++++++++++++++++++++++++{}'.format(self.armarker_pos))

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        self.on_enter(userdata)

    def get_camera_Info(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.info_received = True


    def get_image_Info(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='rgb8')
        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='mono8')

        self.corners, self.marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                             parameters=self.aruco_parameters)
        if self.marker_ids is not None:
            self.get_marker = True
        else:
            self.get_marker = False
