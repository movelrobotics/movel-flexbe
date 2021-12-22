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
from movel_flexbe_states.seirios_run_path_state import SeiriosRunPathState
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
		self.add_parameter('path_name', '')
		self.add_parameter('linear_velocity', 0.2)
		self.add_parameter('angular_velocity', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:754 y:49, x:750 y:160
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:39
			OperatableStateMachine.add('start_log',
										LogState(text="Executing path " + self.path_name, severity=Logger.REPORT_HINT),
										transitions={'done': 'execute_path'},
										autonomy={'done': Autonomy.Off})

			# x:527 y:153
			OperatableStateMachine.add('failed_log',
										LogState(text="Path failed", severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:524 y:46
			OperatableStateMachine.add('success_log',
										LogState(text="Path succeeded", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:274 y:39
			OperatableStateMachine.add('execute_path',
										SeiriosRunPathState(path_name=self.path_name, linear_vel=self.linear_velocity, angular_vel=self.angular_velocity),
										transitions={'arrived': 'success_log', 'failed': 'failed_log'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
