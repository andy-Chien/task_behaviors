'''
Created on 08/23/2021

@author: Frank Lyu
'''
from flexbe_core import EventState, Logger

class ConditionByConditionState(EventState):
    '''
    Set/create userdata value from input userdata use the condition function. (userdata_src_names and userdata_dst_names must be same size.)

    -- condition		function	set function to be used to compute input userdata by condition and return to output userdata.
    -- userdata_names 	string[]	A list containing all userdata key names.
    -- values 			list		Values of userdata

    '''

    def __init__(self, data_condition, out_condition, condition_value, userdata_src_names, userdata_dst_names):
        '''
        Constructor
        '''
        super(ConditionByConditionState, self).__init__(outcomes=['true', 'false'],
                                                input_keys=userdata_src_names,
                                                output_keys=userdata_dst_names)
        self._data_condition = data_condition
        self._out_condition = out_condition
        self._keys_src = userdata_src_names
        self._keys_dst = userdata_dst_names
        self._var_to_check = {'u': None, 'condition_value': condition_value}
        self._success = False

    def execute(self, userdata):
        if not self._success:
            return None
        self._var_to_check['u'] = userdata
        return 'true' if self._out_condition(self._var_to_check) else 'false'

    def on_enter(self, userdata):
        try:
            self._success = False
            if len(self._keys_src) != len(self._keys_dst):
                raise Exception('[ConditionByConditionState] src names and dst names must be same size.')

            for key_src, key_dst in zip(self._keys_src, self._keys_dst):
                userdata[key_dst] = self._data_condition(userdata[key_src])
            self._success = True
        except Exception as e:
            Logger.logerr('Failed to create userdata value:\n%s ' % str(e))
