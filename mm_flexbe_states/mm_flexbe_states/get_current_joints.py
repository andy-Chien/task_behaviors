'''
Created on 04/19/2022
@author: Andy Chien
''' 
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached
from sensor_msgs.msg import JointState

class GetCurrentJoints(EventState):
    '''
    set initial robot collision objects to robot scene
    
    -- namespace        string       robot name or namespace.
    -- joint_names      string[]     joints name of robot

    #> curr_joints      float[]      current joints value

    <= done                     set robot collision objects to initial pose success
    <= no_msg                   no joint state msg
    '''

    def __init__(self, joint_names, namespace=''):
        '''Constructor'''
        super(GetCurrentJoints, self).__init__(outcomes = ['done', 'no_msg'],
                                            output_keys = ['curr_joints'])
        self._node = GetCurrentJoints._node
        ProxyServiceCaller._initialize(self._node)

        self._logger = self._node.get_logger()

        if len(namespace) > 1 or (len(namespace) == 1 and namespace.startswith('/')):
            namespace = namespace[1:] if namespace[0] == '/' else namespace
            self._joint_state_topic = '/' + namespace + '/joint_states'
            self._joint_names = [namespace + '_' + jn for jn in joint_names]
        else:
            self._joint_state_topic = '/joint_states'
            self._joint_names = joint_names

        self._joint_state_sub = ProxySubscriberCached({self._joint_state_topic: JointState})

    def execute(self, userdata):
        if not self._joint_state_sub.has_msg(self._joint_state_topic):
            self._logger.info('self._joint_state_sub.dont has_msg')
            return 'no_msg'
        joint_state = self._joint_state_sub.get_last_msg(self._joint_state_topic)
        userdata.curr_joints = []
        for jn_ in self._joint_names:
            for jn, jp in zip(joint_state.name, joint_state.position):
                if jn == jn_:
                    userdata.curr_joints.append(jp)
                    break
        return 'done'