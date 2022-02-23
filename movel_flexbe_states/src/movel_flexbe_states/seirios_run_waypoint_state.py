#!/usr/bin/env python3

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from movel_seirios_msgs.msg import RunTaskListGoal, RunTaskListAction, Task
from std_msgs.msg import Bool
import json
import requests
import rospy

class SeiriosRunWaypointState(EventState):
    """
    State for waypoint navigation on seirios.

    -- waypoint_name    string      Goal name.
    -- linear_vel       float       Linear velocity.
    -- angular_vel      float       Angular velocity.

    <= arrived                      Waypoint task succeeds, robot's arrived to the destination.
    <= failed                       Waypoint task fails.
    """

    def __init__(self, waypoint_name, linear_vel, angular_vel):
        """Constructor"""
        super(SeiriosRunWaypointState, self).__init__(outcomes = ['arrived', 'failed'])

        self._waypoint_name = waypoint_name
        self._linear_vel = linear_vel
        self._angular_vel = angular_vel

        self._api_address = rospy.get_param('seirios_api/address')
        self._api_username = rospy.get_param('seirios_api/user/username')
        self._api_pwd = rospy.get_param('seirios_api/user/password')

        self._action_topic = '/task_supervisor'
        self._client = ProxyActionClient({self._action_topic: RunTaskListAction})
        
        self._pub = ProxyPublisher({'/task_supervisor/pause': Bool})

        self._completed = False
        self._failed = False

        self._task_supervisor_goal = RunTaskListGoal()


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""
        if self._completed:
            return 'arrived'
        if self._failed:
            return 'failed'
        if self._client.has_result(self._action_topic):
            status = self._client.get_state(self._action_topic)
            if status == GoalStatus.SUCCEEDED:
                self._completed = True
                Logger.loginfo('[%s] Waypoint task completed: %s' % (self.name, str(status)))
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('[%s] Waypoint task failed: %s' % (self.name, str(status)))
                self._failed = True
                return 'failed'


    def on_enter(self, userdata):
        """Create and send action goal"""
        self._completed = False
        self._failed = False

        # retrieve goal coordinate from server
        try:
            login_resp = requests.post('http://%s/api/v1/user/token' % self._api_address,\
                json={"username":self._api_username, "password":self._api_pwd})
            login_resp.raise_for_status()
            
            if login_resp.status_code != 200:
                Logger.logerr('[%s] Unexpected response when requesting auth token: %d' % (self.name, login_resp.status_code))
                self._failed = True
                return

            token = login_resp.json()['token']

            waypoints_resp = requests.get('http://%s/api/v1/waypoint/all' % self._api_address, headers={'authorization':token})
            waypoints_resp.raise_for_status()

            if waypoints_resp.status_code != 200:
                Logger.logerr('[%s] Unexpected response when requesting waypoint list: %d' % (self.name, waypoints_resp.status_code))
                self._failed = True
                return
            
            bot_resp = requests.get('http://%s/api/v1/bot/detail' % self._api_address, headers={'authorization':token})
            bot_resp.raise_for_status()

            if bot_resp.status_code != 200:
                Logger.logerr('[%s] Unexpected response when requesting bot info: %d' % (self.name, waypoints_resp.status_code))
                self._failed = True
                return

            waypoints = waypoints_resp.json()
            bot = bot_resp.json()

            if 'currentMapId' not in bot:
                Logger.logerr('[%s] Robot has no current active map' % self.name)
                self._failed = True
                return

            filtered_waypoints = [waypoint for waypoint in waypoints if waypoint['mapId'] == bot['currentMapId'] and waypoint['name'] == self._waypoint_name]

            if len(filtered_waypoints) != 1:
                Logger.logerr('[%s] Cannot find waypoint with name %s for this active map' % (self.name, self._waypoint_name))
                self._failed = True
                return
            
            waypoint = filtered_waypoints[0]
        except requests.exceptions.RequestException as e:
            Logger.logerr('[%s] Failed to retrieve waypoint data: %s' % (self.name, str(e)))
            self._failed = True
            return

        # construct task supervisor goal
        Logger.loghint('[%s] Constructing message and transmitting goal to task supervisor' % self.name)

        for pose in waypoint['poses']:
            ts_task = Task()
            ts_task.name = 'Goto'
            ts_task.type = 3
            ts_task.mapId = waypoint['mapId']
            
            payload_dict = pose
            payload_dict['from_map'] = ts_task.mapId
            payload_dict['to_map'] = ts_task.mapId
            payload_dict['velocity'] = {'linear_velocity': self._linear_vel, 'angular_velocity': self._angular_vel}

            ts_task.payload = json.dumps(payload_dict)

            ts_task.linear_velocity = self._linear_vel
            ts_task.angular_velocity = self._angular_vel

            self._task_supervisor_goal.task_list.tasks.append(ts_task)

        # Send the action goal for execution
        try:
            self._client.send_goal(self._action_topic, self._task_supervisor_goal)
        except Exception as e:
            Logger.logerr('[%s] Unable to send task supervisor action goal:\n%s' % (self.name, str(e)))
            self._failed = True
            

    def cancel_active_goals(self):
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo('[%s] Cancelled task supervisor active action goal.' % self.name)


    def on_exit(self, userdata):
        self.cancel_active_goals()


    def on_stop(self):
        self.cancel_active_goals()
    

    def on_pause(self):
        Logger.loginfo('[%s] State paused, stopping robot.' % self.name)
        # self.cancel_active_goals()
        pause_msg = Bool()
        pause_msg.data = True
        self._pub.publish('/task_supervisor/pause', pause_msg)


    def on_resume(self, userdata):
        Logger.loginfo('[%s] State resumed, resuming robot navigation.' % self.name)
        pause_msg = Bool()
        pause_msg.data = False
        self._pub.publish('/task_supervisor/pause', pause_msg)
        # try:
        #     self._client.send_goal(self._action_topic, self._task_supervisor_goal)
        # except Exception as e:
        #     Logger.logerr('[%s] Unable to send task supervisor action goal:\n%s' % (self.name, str(e)))
        #     self._failed = True