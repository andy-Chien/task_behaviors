'''
Created on 26/02/2022

@author: Andy Chien
'''
import numpy as np
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose

class SetPlacePoseRandomState(EventState):
    '''
    Set digital IO state

    ># place_position_max  float[]  A list given by [x_max, y_max, z_max] in meter.
    ># place_position_min  float[]  A list given by [x_min, y_min, z_min] in meter.

    #> place_pose          geometry_msg/Pose  The randomed pose.

    <= done                Random done.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(SetPlacePoseRandomState, self).__init__(input_keys=['place_position_max', 'place_position_min'],
                                                      output_keys=['place_pose'],
                                                      outcomes=['done'])

    def execute(self, userdata):
        max, min = np.array(userdata.place_position_max), np.array(userdata.place_position_min)
        if not np.all(max - min >= 0):
            Logger.logerr('[Set Place Pose Random State]: Place Position setting wrong')
        p = Pose()

        pos = (max - min) * np.random.rand(3) + min
        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]
        p.orientation.w = 1.0
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0

        userdata.place_pose = p

        return 'done'
