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

class HiwinXeg32GripperClient(EventState):
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

    <= done                     set robot collision objects to initial pose success
    '''

    def __init__(self, mode='open', direction=0, distance=0, speed=0, 
                 holding_stroke=0, holding_speed=0, holding_force=0, flag=0, namespace='', sim=False):
        '''Constructor'''
        super(HiwinXeg32GripperClient, self).__init__(outcomes = ['done', 'failed'])
        
        ProxyServiceCaller._initialize(EventState._node)
        self._logger = EventState._node.get_logger().info

        self._sim = sim
        if sim:
            return

        if 'on' in mode:
            cmd_mode=GripperCommand.Request.ON
        elif 'off' in mode:
            cmd_mode=GripperCommand.Request.OFF
        elif 'reset' in mode:
            cmd_mode=GripperCommand.Request.RESET
        elif 'close' in mode:
            cmd_mode=GripperCommand.Request.CLOSE
        elif 'open' in mode:
            cmd_mode=GripperCommand.Request.OPEN
        elif 'move' in mode:
            cmd_mode=GripperCommand.Request.MOVE
        elif 'expert' in mode:
            cmd_mode=GripperCommand.Request.EXPERT

        self.req = self.generate_gripper_request(
            cmd_mode=cmd_mode,
            direction=direction,
            distance=distance,
            speed=speed,
            holding_stroke=holding_stroke,
            holding_speed=holding_speed,
            holding_force=holding_force,
            flag=flag
        )

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self.gripper_service = '/' + namespace + '/execute_gripper'
        else:
            self.gripper_service = '/execute_gripper'
        self.srv_ava = True
        
        self._gripper_client = ProxyServiceCaller({self.gripper_service : GripperCommand})

    def execute(self, _):
        if self._sim:
            return 'done'
        if not self.srv_ava:
            self.srv_ava = True
            return 'failed'
        if not self._gripper_client.done(self.gripper_service):
            self._logger('waiting gripper')
            return
        result = self._gripper_client.result(self.gripper_service)
        # self._logger(result)
        gs = result.gripper_state
        print(gs)
        gr = GripperCommand.Response
        if gs == gr.IDLE or gs == gr.HOLD or gs == gr.POSITION:
            return 'done'
        else:
            return 'failed'

    def on_enter(self, _):
        if self._sim:
            return
        if not self._gripper_client.is_available(self.gripper_service):
            self.srv_ava = False
            return
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
        if cmd_mode != GripperCommand.Request.EXPERT or \
                cmd_mode != GripperCommand.Request.MOVE:
            direction, distance, speed, holding_stroke = 0, 0, 0, 0
            holding_speed, holding_force, flag = 0, 0, 1
        if cmd_mode == GripperCommand.Request.MOVE:
            holding_stroke, holding_speed, holding_force, flag = 0, 0, 0, 1

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
    
        
