##! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from orion_hri.hri import HRI

class Prompt(smach.State):
    """
    Prompt

    """

    def __init__(self, question, valid_sentences, valid_objects_with_word, sentence_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['arguments', 'objects', 'neg_objects'],
                             output_keys=['arguments'])

        self.question = question
        self.valid_sentences = valid_sentences
        self.valid_objects_with_word = valid_objects_with_word
        self.sentence_not_valid = sentence_not_valid
        
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("STATE - Prompt")

        userdata.arguments = []

        valid_sentences = []
        rospy.loginfo('Current objects: %s', str(userdata.objects))
        rospy.loginfo('Current neg_objects: %s', str(userdata.neg_objects))
        for s in self.valid_sentences:
            words = s.split(' ')
            if len(words) == 4 :
                obj = str(words[-1])
            elif len(words) == 5 :
                obj = str(words[-2]) + ' ' + str(words[-1])
            else:
                obj = ''

                
            if not (obj in userdata.objects or obj in userdata.neg_objects):
                valid_sentences.append(s)


        (sentences,scores) = self.hri.prompt(self.question, valid_sentences, self.sentence_not_valid)


        #exp_sentences = _expand_sentences(sentences, self.valid_objects_with_word)
        
        for sentence in sentences:
            words = sentence.split(' ')
            print(sentence, words)
            if len(words) == 4: # bring me the OBJECT
                userdata.arguments.append(str(words[-1]))
            elif len(words) == 5: # bring me the PROPERTY OBJECT
                userdata.arguments.append(str(words[-2] + ' ' + words[-1]))
            else:
                rospy.logerror('Invalid sentence: %s', sentence)
            
            #self.hri.say("I understood: " +  sentence)
        
        return 'succeeded'


class PromptInput(smach.State):
    """
    Prompt

    """

    def __init__(self, question, possible_inputs, input_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['input', 'arguments'],
                             output_keys=['input', 'arguments'])

        self.question = question
        self.possible_inputs = possible_inputs
        self.input_not_valid = input_not_valid
        
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("STATE - PromptInput")

        (sentences,scores) = self.hri.prompt(self.question, self.possible_inputs, self.input_not_valid)

        for sentence in sentences:            
            self.hri.say("I understood: " +  sentence)

        # most likely input
        userdata.input = sentences[0]
        userdata.arguments = []
        userdata.arguments.append(sentences[0])
            
        return 'succeeded'
    
