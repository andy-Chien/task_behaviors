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

    -- place_position_max  float[]  A list given by [x_max, y_max, z_max] in meter.
    -- place_position_min  float[]  A list given by [x_min, y_min, z_min] in meter.
    -- place_quaternion    float[]  The quaternion of place pose given by [w, x, y, z].

    #> place_position      float[]  The position of place pose given by [x, y, z] in meter.
    #> place_quaternion    float[]  The quaternion of place pose given by [w, x, y, z].

    <= done                Random done.
    '''

    def __init__(self, place_position_max, place_position_min, place_quaternion):
        '''
        Constructor
        '''
        super(SetPlacePoseRandomState, self).__init__(output_keys=['place_position', 'place_quaternion'],
                                                      outcomes=['done'])
        self._max = np.array(place_position_max)
        self._min = np.array(place_position_min)
        self._place_quaternion = place_quaternion
        if not np.all(self._max - self._min > 0):
            rospy.logerr('[Set Place Pose Random State]: Place Position setting wrong')

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        userdata.place_quaternion = self._place_quaternion
        userdata.place_position = (self._max - self._min) * np.random.rand(3) + self._min