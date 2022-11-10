#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import dynamic_reconfigure.client
import rospy

class ToggleCostmapLayerState(EventState):
    '''
    Toggle costmap layer on or off.

    -- costmap_name string  The costmap name.
    -- layer        string  Costmap layer to be enabled/disabled.
    -- enable       bool    Enable or disable costmap layer.

    <= done     Done.
    <= failed   Failed.
    '''

    def __init__(self, costmap_name, layer, enable):
        super(ToggleCostmapLayerState, self).__init__(outcomes=['done', 'failed'])
        self._costmap_name = costmap_name.rstrip("/")
        self._layer = layer
        self._enable = enable
        self._updated = False


    def execute(self, userdata):
        if self._enable:
            Logger.loghint("[%s] Enabling layer %s at costmap %s" % (self.name, self._layer, self._costmap_name))
        else:
            Logger.loghint("[%s] Disabling layer %s at costmap %s" % (self.name, self._layer, self._costmap_name))

        try:
            client = dynamic_reconfigure.client.Client(self._costmap_name + "/" + self._layer, timeout=5, config_callback=self.callback)
            client.update_configuration({"enabled": self._enable})
        except Exception as e:
            Logger.logerr("[%s] Error: %s" % (self.name, e))
            return 'failed'

        if self._enable:
            Logger.loghint("[%s] Costmap layer enabled!" % self.name)
        else:
            Logger.loghint("[%s] Costmap layer disabled!" % self.name)
        return 'done'

    def on_enter(self, userdata):
        Logger.loghint('[%s] Entering %s state' % (self.name, self.name))
    
    def callback(self, config):
        self._updated = True