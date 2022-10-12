#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import dynamic_reconfigure.client
import rospy

class ToggleCostmapLayerState(EventState):
    '''
    Publishes a mutex area tb_agv_msgs/Mutex message on a given topic name.

    -- costmap_node string The costmap publishing node.

    -- layer string Costmap layer to be enabled/disabled.
   
    -- enable    bool                Enable or disable costmap layer  

    <= done Done.
    '''

    def __init__(self, layer, enable, costmap_node="/move_base/local_costmap"):
        super(ToggleCostmapLayerState, self).__init__(outcomes=['done'])
        self._costmap_node = costmap_node.rstrip("/")
        self._layer = layer
        self._enable = enable
        self._updated = False


    def execute(self, userdata):
        client = dynamic_reconfigure.client.Client(self._costmap_node + "/" + self._layer, timeout=30, config_callback=self.callback)
        client.update_configuration({"enabled": self._enable})

        while not self._updated:
            rospy.sleep(0.05)

        if self._enable:
            Logger.loghint("[%s] Costmap layer enabled!" % self.name)
        else:
            Logger.loghint("[%s] Costmap layer disabled!" % self.name)
        return 'done'

    def on_enter(self, userdata):
        Logger.loghint('[%s] Entering %s state' % (self.name, self.name))
    
    def callback(self, config):
        self._updated = True