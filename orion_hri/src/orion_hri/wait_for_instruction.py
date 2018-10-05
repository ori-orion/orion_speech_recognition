#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

from orion_hri.setup import Setup
from orion_hri.listen import Listen
from orion_hri.confirm import Confirm

from orion_hri.recognize_speech import RecognizeSpeech
from orion_hri.ask_confirm import AskConfirm
from orion_hri.ask_help import AskHelp
from orion_hri.shutdown import Shutdown

import numpy
import tf

class WaitForInstructionSM(smach.StateMachine):
    def __init__(self, mode):
        smach.StateMachine.__init__(self, outcomes=['succeeded',
                                                    'aborted',
                                                    'preempted'])

        cmd_question = "Hello, what can I bring you?"
        cmd_confirmation = "Should I bring you the "
        cmd_more_objs = "Should I bring you more objects?"
        
        valid_objs = ['bring me the box',
                      'bring me the bottle',
                      'bring me the cup',
                      'bring me the sponge']
        confirmations = ['yes please']
        rejections = ['no thanks']
        
        self.userdata.action_completed = False
        self.userdata.objects = []

        self._setup  = Setup()
        self._listen_for_cmd = Listen(cmd_question, valid_objs)
        self._confirm_cmd = Confirm(cmd_confirmation, False, confirmations, rejections)
        self._confirm_more_objects = Confirm(cmd_more_objs, True, confirmations, rejections)

        # self._recognize_speech = RecognizeSpeech()
        # self._ask_confirm = AskConfirm()
        # self._ask_help = AskHelp()
        
        self._shutdown = Shutdown()


        with self:
            smach.StateMachine.add('Setup', self._setup,
                                   transitions={'succeeded': 'ListenForCommand',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ListenForCommand', self._listen_for_cmd,
                                   transitions={'succeeded': 'ConfirmCommand',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ConfirmCommand', self._confirm_cmd,
                                   transitions={'succeeded': 'ConfirmMoreObjects',
                                                'not_confirmed': 'ListenForCommand',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})

            smach.StateMachine.add('ConfirmMoreObjects', self._confirm_more_objects,
                                   transitions={'succeeded': 'ListenForCommand',
                                                'not_confirmed': 'Shutdown',
                                                'aborted':'Shutdown',
                                                'preempted':'Shutdown'})
            

            # smach.StateMachine.add('AskConfirm', self._ask_confirm,
            #                        transitions={'succeeded': 'Listen',
            #                                     'aborted': 'Shutdown',
            #                                     'preempted':'Shutdown'})

            # smach.StateMachine.add('AskHelp', self._ask_help,
            #                        transitions={'succeeded':'Listen',
            #                                     'aborted':'Shutdown',
            #                                     'preempted':'Shutdown'})

            smach.StateMachine.add('Shutdown', self._shutdown,
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
