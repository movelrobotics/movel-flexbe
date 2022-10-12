#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from movel_flexbe_states.dynamic_reconfigure_state import DynamicReconfigureState
from movel_flexbe_states.toggle_costmap_layer_state import ToggleCostmapLayerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 29 2022
@author: test
'''
class costmap_testSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(costmap_testSM, self).__init__()
		self.name = 'costmap_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:443 y:125, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:191 y:185
			OperatableStateMachine.add('dyn',
										DynamicReconfigureState(parameter_dict={"footprint": [[0.135,0.135],[0.135,-0.135],[-0.135,-0.135],[-0.135,0.135]]}, reconfigure_node="/move_base/global_costmap"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:173 y:100
			OperatableStateMachine.add('costmap',
										ToggleCostmapLayerState(layer="/obstacle_layer", enable=True, costmap_node="/move_base/local_costmap"),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
