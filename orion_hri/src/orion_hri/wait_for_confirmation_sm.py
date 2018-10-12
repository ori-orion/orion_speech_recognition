#! /usr/bin/env python
import rospy
import smach
import smach_ros

from orion_hri.setup import Setup
from orion_hri.prompt import Prompt
from orion_hri.confirm import Confirm
from orion_hri.confirm import ConfirmInput
from orion_hri.shutdown import Shutdown

class WaitForConfirmationSM(smach.StateMachine):
    def __init__(self, question, positive_answers, negative_answers, timeout):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])
        self.question = question
        self.positive_answers = positive_answers
        self.negative_answers = negative_answers
                
        self.userdata.is_confirmed = False

        self._confirm = Confirm(self.question, self.positive_answers, self.negative_answers, '')

        with self:
            smach.StateMachine.add('Confirm', self._confirm,
                                   transitions={'succeeded': 'succeeded',
                                                'not_confirmed': 'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
