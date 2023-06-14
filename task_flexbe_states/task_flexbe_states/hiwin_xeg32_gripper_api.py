'''
Created on 06/12/2023
@author: TaiTing Tsai
''' 
import os
import sys   
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from ctypes import CDLL
import time
import rclpy
from ament_index_python.packages import get_package_share_directory
from hiwin_gripper_interfaces.srv import GripperCommand

class HiwinXeg32GripperApi(EventState):
    '''
    set initial robot collision objects to robot scene
    
    -- mode            string   gripper mode
    -- direction       int      move direction of gripper 0 is close 1 is open, only use in expert mode
    -- distance        int      move distance of gripper 
    -- speed           int      move distance of gripper
    -- holding_stroke  int      move distance after finished move distance, only use in expert mode
    -- holding_speed   int      move speed after finished move distance, only use in expert mode
    -- holding_force   int      force of gripper, only use in expert mode
    -- flag

    ># init_joints      float[]      initial joints value for IK compute

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self):
        '''Constructor'''
        super(HiwinXeg32GripperApi, self).__init__(outcomes = ['done', 'failed'],
                                            input_keys = ['mode', 'direction', 'distance', 'speed','holding_stroke', 
                                                          'holding_speed', 'holding_force', 'flag'])
        
        self._node = HiwinXeg32GripperApi._node
        ProxyServiceCaller._initialize(self._node)
        self._logger = self._node.get_logger().info
        self.mode = None
        self.direction = None
        self.distance = None
        self.speed = None
        self.holding_stroke = None
        self.holding_speed = None
        self.holding_force = None
        self.flag = None
        self.req = GripperCommand.Request()
        
        self.gripper_service = '/execute_gripper'
        self._gripper_client = ProxyServiceCaller({self.gripper_service : GripperCommand})

        self._time_now = self._node.get_clock().now()

    def execute(self, userdata):
        # if 'on' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.ON,
        #     )
        # elif 'reset' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.RESET,
        #     )
        # elif 'close' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.CLOSE,
        #     )
        # elif 'open' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.OPEN,
        #     )
        # elif 'move' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.MOVE,
        #         distance=self.distance,
        #         speed=self.speed
        #     )
        # elif 'expert' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.EXPERT,
        #         direction=self.direction,
        #         distance=self.distance,
        #         speed=self.speed,
        #         holding_stroke=self.holding_stroke,
        #         holding_speed=self.holding_speed,
        #         holding_force=self.holding_force,
        #     )            
        # elif 'off' in self.mode:
        #     self.req = self.generate_gripper_request(
        #         cmd_mode=GripperCommand.Request.OFF,
        #     )            
        # self._gripper_client.call_async(self.gripper_service, self.req)
        if not self._gripper_client.done(self.gripper_service):
            self._logger('waiting gripper')
            return
        result = self._gripper_client.result(self.gripper_service)
        # self._logger(result)
        print(result.gripper_state)
        if result.gripper_state == GripperCommand.Response.IDLE or result.gripper_state == GripperCommand.Response.HOLD or result.gripper_state == GripperCommand.Response.POSITION:
            return 'done'
        else:
            return 'failed'

    def on_enter(self, userdata):
        self.mode = userdata.mode
        if userdata.direction != None:
            self.direction = userdata.direction
        if userdata.distance != None:
            self.distance = userdata.distance
        if userdata.speed != None:
            self.speed = userdata.speed
        if userdata.holding_stroke != None:
            self.holding_stroke = userdata.holding_stroke
        if userdata.holding_speed != None:
            self.holding_speed = userdata.holding_speed
        if userdata.holding_force != None:
            self.holding_force = userdata.holding_force
        if userdata.flag != None:
            self.flag = userdata.flag

        if 'on' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.ON,
            )
        elif 'reset' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.RESET,
            )
        elif 'close' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.CLOSE,
            )
        elif 'open' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.OPEN,
            )
        elif 'move' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.MOVE,
                distance=self.distance,
                speed=self.speed
            )
        elif 'expert' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.EXPERT,
                direction=self.direction,
                distance=self.distance,
                speed=self.speed,
                holding_stroke=self.holding_stroke,
                holding_speed=self.holding_speed,
                holding_force=self.holding_force,
            )            
        elif 'off' in self.mode:
            self.req = self.generate_gripper_request(
                cmd_mode=GripperCommand.Request.OFF,
            )    
        self._gripper_client.call_async(self.gripper_service, self.req)     

    def generate_gripper_request(
            self, 
            cmd_mode=GripperCommand.Request.OPEN,
            direction=0,
            distance=0,
            speed=0,
            holding_stroke=0,
            holding_speed=0,
            holding_force=0,
            flag=1
            ):
        request = GripperCommand.Request()
        request.cmd_mode = cmd_mode
        request.direction = direction
        request.distance = distance
        request.speed = speed
        request.holding_stroke = holding_stroke
        request.holding_speed = holding_speed
        request.holding_force = holding_force
        request.flag = flag
        return request
    
        
