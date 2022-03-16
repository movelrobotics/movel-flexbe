#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from movel_flexbe_states.seirios_run_navigation_state import SeiriosRunNavigationState
from movel_flexbe_states.seirios_run_waypoint_state import SeiriosRunWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Dec 03 2021
@author: Movel AI
'''
class movel_navigationSM(Behavior):
	'''
	movel_navigation
	'''


	def __init__(self):
		super(movel_navigationSM, self).__init__()
		self.name = 'movel_navigation'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:821 y:59, x:821 y:171
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:73 y:29
			OperatableStateMachine.add('start_log',
										LogState(text="Starting navigation to Goal", severity=Logger.REPORT_HINT),
										transitions={'done': 'waypoint'},
										autonomy={'done': Autonomy.Off})

			# x:258 y:27
			OperatableStateMachine.add('navigation',
										SeiriosRunNavigationState(goal_name='WAYPOINT_23-02-22_01', linear_vel=0.2, angular_vel=0.2),
										transitions={'arrived': 'success_log', 'failed': 'failed_log'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:563 y:37
			OperatableStateMachine.add('success_log',
										LogState(text="Navigation succeeded", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:266 y:118
			OperatableStateMachine.add('waypoint',
										SeiriosRunWaypointState(waypoint_name='CENTER_LOOP', linear_vel=0.3, angular_vel=0.3, start_at_nearest_point=False),
										transitions={'arrived': 'success_log', 'failed': 'failed_log'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:565 y:161
			OperatableStateMachine.add('failed_log',
										LogState(text="Navigation failed", severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
