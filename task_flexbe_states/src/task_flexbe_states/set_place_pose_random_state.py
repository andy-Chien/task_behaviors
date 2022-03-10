'''
Created on 26/02/2022

@author: Andy Chien
'''
import rospy
import numpy as np
from flexbe_core import EventState, Logger

class SetPlacePoseRandomState(EventState):
    '''
    Set digital IO state

    ># place_position_max  float[]  A list given by [x_max, y_max, z_max] in meter.
    ># place_position_min  float[]  A list given by [x_min, y_min, z_min] in meter.

    #> place_position      float[]  The position of place pose given by [x, y, z] in meter.

    <= done                Random done.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(SetPlacePoseRandomState, self).__init__(input_keys=['place_position_max', 'place_position_min'],
                                                      output_keys=['place_position'],
                                                      outcomes=['done'])

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        max, min = np.array(userdata.place_position_max), np.array(userdata.place_position_min)
        if not np.all(max - min >= 0):
            rospy.logerr('[Set Place Pose Random State]: Place Position setting wrong')
        userdata.place_position = (max - min) * np.random.rand(3) + min