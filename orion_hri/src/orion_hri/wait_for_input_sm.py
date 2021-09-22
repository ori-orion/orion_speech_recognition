#! /usr/bin/env python3
import rospy
import smach
import smach_ros

from orion_hri.prompt import PromptInput
from orion_hri.confirm import ConfirmInput

class WaitForInputSM(smach.StateMachine):
    def __init__(self, question, possible_inputs, timeout):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])
        self.question = question
        self.possible_inputs = possible_inputs


        self.confirm_input = 'Did you say '

        self.positive_ex = ['yes correct']
        self.negative_ex = ['no cancel']
        
        self.userdata.input = ''
        self.userdata.objects = []
        self.userdata.neg_objects = []


        
        self._prompt = PromptInput(self.question, self.possible_inputs, '')
        self._confirm = ConfirmInput(self.confirm_input, self.positive_ex, self.negative_ex, '')

        with self:
            smach.StateMachine.add('PromptInput', self._prompt,
                                   transitions={'succeeded': 'ConfirmInput',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('ConfirmInput', self._confirm,
                                   transitions={'succeeded': 'succeeded',
                                                'not_confirmed': 'PromptInput',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
