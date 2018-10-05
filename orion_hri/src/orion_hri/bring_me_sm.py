#! /usr/bin/env python
import rospy
import smach
import smach_ros

from orion_hri.setup import Setup
from orion_hri.prompt import Prompt
from orion_hri.confirm import Confirm
from orion_hri.confirm import ConfirmInput
from orion_hri.shutdown import Shutdown

class BringMeSM(smach.StateMachine):
    def __init__(self, mode):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        obj_question = "What can I bring you?"
        obj_confirmation = "Should I bring you the "
        obj_another = "Should I bring you more objects?"
        
        valid_objs = ['bring me the box',
                      'bring me the bottle',
                      'bring me the cup',
                      'bring me the sponge']
        positive_ex = ['yes please']
        negative_ex = ['no thanks']
        
        self.userdata.objects = []

        self._setup  = Setup()
        self._prompt_for_obj = Prompt(obj_question, valid_objs)
        self._confirm_obj = ConfirmInput(obj_confirmation, positive_ex, negative_ex)
        self._confirm_another_obj = Confirm(obj_another, positive_ex, negative_ex)
        self._shutdown = Shutdown()


        with self:
            smach.StateMachine.add('Setup', self._setup,
                                   transitions={'succeeded': 'PromptForObject',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('PromptForObject', self._prompt_for_obj,
                                   transitions={'succeeded': 'ConfirmObject',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ConfirmObject', self._confirm_obj,
                                   transitions={'succeeded': 'ConfirmAnotherObject',
                                                'not_confirmed': 'PromptForObject',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ConfirmAnotherObject', self._confirm_another_obj,
                                   transitions={'succeeded': 'PromptForObject',
                                                'not_confirmed': 'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})
            
            smach.StateMachine.add('Shutdown', self._shutdown,
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
