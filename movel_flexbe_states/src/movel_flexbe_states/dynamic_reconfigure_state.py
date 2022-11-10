#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import dynamic_reconfigure.client
import rospy

class DynamicReconfigureState(EventState):
    '''
    Modify parameter values at runtime with Dynamic Reconfigure (only for parameters with Dynamic Reconfigure enabled).

    -- reconfigure_namespace    string  The namespace in which the parameter is located.
    -- parameter_dict           dict    The dynamically-reconfigurable parameter and value pairs.

    <= done     Done.
    <= failed   Failed.
    '''

    def __init__(self, reconfigure_namespace, parameter_dict):
        super(DynamicReconfigureState, self).__init__(outcomes=["done", "failed"])
        self._reconfigure_namespace = reconfigure_namespace.rstrip("/")
        self._parameter_dict = parameter_dict
        self._updated = False

    def execute(self, userdata):
        Logger.loghint("[{}] Updating parameters in namespace {}: {}".format(self.name, self._reconfigure_namespace, self._parameter_dict))

        try:
            client = dynamic_reconfigure.client.Client(self._reconfigure_namespace, timeout=5, config_callback=self.callback)
            client.update_configuration(self._parameter_dict)
        except Exception as e:
            Logger.logerr("[%s] Error: %s" % (self.name, e))
            return 'failed'

        Logger.loghint("[%s] Parameters successfully updated." % (self.name))

        return "done"

    def on_enter(self, userdata):
        Logger.loghint("[%s] Entering %s state" % (self.name, self.name))
    
    def callback(self, config):
        self._updated = True