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
        obj_repeat = "Sorry, I did not understand what you said."

        prefix = 'bring me the '
        objs = ['Wooden Bowl',
                'Brown pail',
                'Mixed nuts',
                'Blue cup',
                'Pink cup',
                'Orange biscuits',
                'Yellow biscuits',
                'Watering can',
                'Flashlight',
                'Blue fork',
                'Green fork',
                'Gray knife',
                'Pink biscuits',
                'Yellow clock',
                'Aluminum foil',
                'Green dish',
                'Yellow dish',
                'Pink bowl',
                'Blue bowl',
                'Mineral water',
                'Oolong Tea',
                'Ketchup',
                'Canned mustard',
                'Orange drink',
                'Green drink',
                'Tomato',
                'Potato',
                'Plant',
                'Spray',
                'Water bottle',
                'Blueberry drink',
                'Eggplant']

        valid_objs = []
        for o in objs:
            valid_objs.append(prefix + o.lower())

        print(valid_objs)
        
        positive_ex = ['yes please']
        negative_ex = ['no thanks']
        
        self.userdata.objects = []

        self._setup  = Setup()
        self._prompt_for_obj = Prompt(obj_question, valid_objs, obj_repeat)
        self._confirm_obj = ConfirmInput(obj_confirmation, positive_ex, negative_ex, obj_repeat)
        self._confirm_another_obj = Confirm(obj_another, positive_ex, negative_ex, obj_repeat)
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
