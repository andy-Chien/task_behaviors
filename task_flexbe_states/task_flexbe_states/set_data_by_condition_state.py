'''
Created on 08/23/2021

@author: Frank Lyu
'''
from flexbe_core import EventState, Logger

class SetDataByConditionState(EventState):
    '''
    Set/create userdata value from input userdata use the condition function. (userdata_src_names and userdata_dst_names must be same size.)

    -- condition		function	set function to be used to compute input userdata by condition and return to output userdata.
    -- userdata_names 	string[]	A list containing all userdata key names.
    -- values 			list		Values of userdata

    '''

    def __init__(self, condition, userdata_src_names, userdata_dst_names):
        '''
        Constructor
        '''
        super(SetDataByConditionState, self).__init__(outcomes=['done'],
                                                input_keys=userdata_src_names,
                                                output_keys=userdata_dst_names)
        self._condition = condition
        self._keys_src = userdata_src_names
        self._keys_dst = userdata_dst_names
        self.success = False

    def execute(self, userdata):
        return 'done' if self.success else None

    def on_enter(self, userdata):
        try:
            self.success = False
            if len(self._keys_src) != len(self._keys_dst):
                raise Exception('[SetDataByConditionState] src names and dst names must be same size.')

            for key_src, key_dst in zip(self._keys_src, self._keys_dst):
                userdata[key_dst] = self._condition(userdata[key_src])
            self.success = True
        except Exception as e:
            Logger.logerr('Failed to create userdata value:\n%s ' % str(e))
