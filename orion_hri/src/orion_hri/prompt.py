#! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from orion_hri.hri import HRI

class Prompt(smach.State):
    """
    Listen

    """

    def __init__(self, question, valid_sentences):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=['argument'])

        self.question = question
        self.valid_sentences = valid_sentences
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("Prompt")

        (sentence,score) = self.hri.prompt(self.question, self.valid_sentences)
        
        userdata.argument = sentence.split(' ')[-1]
        self.hri.say("I understood: " +  sentence)
        
        return 'succeeded'
