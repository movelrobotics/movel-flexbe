#/usr/bin/env python3

import rospy
from std_msgs.msg import String
from flexbe_msgs.msg import BehaviorLog

STATUS_CODE = {
    0: 'INFO',
    1: 'WARN',
    2: 'HINT',
    3: 'ERROR',
    10: 'DEBUG'
}

class FlexbeMsgRepublisher:
    def __init__(self):
        self._flexbe_log_sub = rospy.Subscriber('/flexbe/log', BehaviorLog, self._flexbe_log_cb)
        self._flexbe_log_string_pub = rospy.Publisher('/flexbe/log/string', String, queue_size=1)

        rospy.on_shutdown(self._shutdown_handler)


    def _flexbe_log_cb(self, behavior_log):
        log_msg = behavior_log.text
        status_code = behavior_log.status_code

        flexbe_log_str = String()
        flexbe_log_str.data = '[{}] {}'.format(STATUS_CODE[status_code], log_msg)

        self._flexbe_log_string_pub.publish(flexbe_log_str)
    

    def _shutdown_handler(self):
        pass


if __name__ == '__main__':
    rospy.init_node('flexbe_message_republisher', anonymous=True)
    flexbe_msg_repub = FlexbeMsgRepublisher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down node')