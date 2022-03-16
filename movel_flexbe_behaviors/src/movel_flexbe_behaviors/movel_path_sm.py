#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
from movel_flexbe_states.seirios_run_path_state import SeiriosRunPathState
from movel_flexbe_states.seirios_run_trail_state import SeiriosRunTrailState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 07 2021
@author: Movel AI
'''
class movel_pathSM(Behavior):
	'''
	Movel Path
	'''


	def __init__(self):
		super(movel_pathSM, self).__init__()
		self.name = 'movel_path'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:865 y:216, x:874 y:52
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:39
			OperatableStateMachine.add('start_log',
										LogState(text='Executing path', severity=Logger.REPORT_HINT),
										transitions={'done': 'wait_1'},
										autonomy={'done': Autonomy.Off})

			# x:419 y:55
			OperatableStateMachine.add('execute_path',
										SeiriosRunPathState(path_name='PATH_28-01-22_01', linear_vel=0.2, angular_vel=0.2),
										transitions={'arrived': 'finished', 'failed': 'failed_log'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:636 y:44
			OperatableStateMachine.add('failed_log',
										LogState(text='Path failed', severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:411 y:231
			OperatableStateMachine.add('subscriber_1',
										SubscriberState(topic='/repeat_path', blocking=True, clear=True),
										transitions={'received': 'conditional', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:422 y:352
			OperatableStateMachine.add('success_log',
										LogState(text='Path succeeded', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:422 y:134
			OperatableStateMachine.add('trail',
										SeiriosRunTrailState(trail_name='TRAIL_23-02-22_01', linear_vel=0.3, angular_vel=0.3, start_at_nearest_point=False),
										transitions={'arrived': 'failed', 'failed': 'finished'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})

			# x:240 y:41
			OperatableStateMachine.add('wait_1',
										WaitState(wait_time=5.0),
										transitions={'done': 'trail'},
										autonomy={'done': Autonomy.Off})

			# x:195 y:234
			OperatableStateMachine.add('conditional',
										CheckConditionState(predicate=lambda x:x.data==True),
										transitions={'true': 'start_log', 'false': 'success_log'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
