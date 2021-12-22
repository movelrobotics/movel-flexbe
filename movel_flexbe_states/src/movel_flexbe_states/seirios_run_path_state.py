#!/usr/bin/env python3

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from actionlib_msgs.msg import GoalStatus
from movel_seirios_msgs.msg import RunTaskListGoal, RunTaskListAction, Task
from std_msgs.msg import Bool
import json
import requests
import rospy

class SeiriosRunPathState(EventState):
    """
    State for path execution on seirios.

    -- path_name    string      Path name.
    -- linear_vel   float       Linear velocity.
    -- angular_vel  float       Angular velocity.

    <= arrived                  Navigation task succeeds, robot's arrived to the destination.
    <= failed                   Navigation task fails.
    """

    def __init__(self, path_name, linear_vel, angular_vel):
        """Constructor"""
        super(SeiriosRunPathState, self).__init__(outcomes = ['arrived', 'failed'])

        self._path_name = path_name
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
                Logger.loginfo('[%s] Navigation completed: %s' % (self.name, str(status)))
                return 'arrived'
            elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED,
                            GoalStatus.RECALLED, GoalStatus.ABORTED]:
                Logger.logwarn('[%s] Navigation failed: %s' % (self.name, str(status)))
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

            paths_resp = requests.get('http://%s/api/v1/path/all' % self._api_address, headers={'authorization':token})
            paths_resp.raise_for_status()

            if paths_resp.status_code != 200:
                Logger.logerr('[%s] Unexpected response when requesting path list: %d' % (self.name, paths_resp.status_code))
                self._failed = True
                return
            
            bot_resp = requests.get('http://%s/api/v1/bot/detail' % self._api_address, headers={'authorization':token})
            bot_resp.raise_for_status()

            if bot_resp.status_code != 200:
                Logger.logerr('[%s] Unexpected response when requesting bot info: %d' % (self.name, bot_resp.status_code))
                self._failed = True
                return

            paths = paths_resp.json()
            bot = bot_resp.json()

            if 'currentMapId' not in bot:
                Logger.logerr('[%s] Robot has no current active map' % self.name)
                self._failed = True
                return

            filtered_paths = [path for path in paths if path['mapId'] == bot['currentMapId'] and path['name'] == self._path_name]

            if len(filtered_paths) != 1:
                Logger.logerr('[%s] Cannot find path with name %s for this active map' % (self.name, self._path_name))
                self._failed = True
                return
            
            path = filtered_paths[0]
        except requests.exceptions.RequestException as e:
            Logger.logerr('[%s] Failed to retrieve path data: %s' % (self.name, str(e)))
            self._failed = True
            return

        # construct task supervisor goal
        Logger.loghint('[%s] Constructing message and transmitting goal to task supervisor' % self.name)

        # task 1: Goto first point
        goto_task = Task()
        goto_task.name = 'Goto'
        goto_task.type = 3
        goto_task.mapId = path['mapId']
        
        goto_payload_dict = path['path'][0]
        goto_payload_dict['from_map'] = goto_task.mapId
        goto_payload_dict['to_map'] = goto_task.mapId

        goto_task.payload = json.dumps(goto_payload_dict)

        goto_task.linear_velocity = self._linear_vel
        goto_task.angular_velocity = self._angular_vel

        self._task_supervisor_goal.task_list.tasks.append(goto_task)

        # task 2: Execute path
        path_task = Task()
        path_task.name = 'Path'
        path_task.type = 6
        path_task.mapId = path['mapId']

        path_payload_dict = {'path': path['path']}
        for i in range(len(path_payload_dict['path'])):
            path_payload_dict['path'][i]['from_map'] = path_task.mapId
            path_payload_dict['path'][i]['to_map'] = path_task.mapId
        
        path_task.payload = json.dumps(path_payload_dict)

        path_task.linear_velocity = self._linear_vel
        path_task.angular_velocity = self._angular_vel

        self._task_supervisor_goal.task_list.tasks.append(path_task)

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