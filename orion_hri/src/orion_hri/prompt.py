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

    def __init__(self, question, valid_sentences, sentence_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=['argument'])

        self.question = question
        self.valid_sentences = valid_sentences
        self.sentence_not_valid = sentence_not_valid
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("Prompt")

        (sentence,score) = self.hri.prompt(self.question, self.valid_sentences, self.sentence_not_valid)

        words = sentence.split(' ')
        print(sentence, words)
        if len(words) == 4: # bring me the OBJECT
            userdata.argument = str(words[-1])
        elif len(words) == 5: # bring me the PROPERTY OBJECT
            userdata.argument = str(words[-2] + ' ' + words[-1])
        else:
            rospy.logerror('Invalid sentence: %s', sentence)
            
        self.hri.say("I understood: " +  sentence)
        
        return 'succeeded'
