#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import dynamic_reconfigure.client
import rospy

class DynamicReconfigureState(EventState):
    '''
    Publishes a mutex area tb_agv_msgs/Mutex message on a given topic name.

    -- parameter_dict       dict    The dynamically-reconfigurable parameter and value pairs.

    -- reconfigure_node     string  The node where the parameter is used.

    <= done Done.
    '''

    def __init__(self, parameter_dict, reconfigure_node):
        super(DynamicReconfigureState, self).__init__(outcomes=["done"])
        self._reconfigure_node = reconfigure_node.rstrip("/")
        self._parameter_dict = parameter_dict
        self._updated = False

    def execute(self, userdata):
        Logger.loghint("[{}] Updating parameters at node {}: {}".format(self.name, self._reconfigure_node, self._parameter_dict))

        client = dynamic_reconfigure.client.Client(self._reconfigure_node, timeout=30, config_callback=self.callback)
        client.update_configuration(self._parameter_dict)

        while not self._updated:
            rospy.sleep(0.1)

        Logger.loghint("[%s] Successfully updating parameters." % (self.name))

        return "done"

    def on_enter(self, userdata):
        Logger.loghint("[%s] Entering %s state" % (self.name, self.name))
    
    def callback(self, config):
        self._updated = True