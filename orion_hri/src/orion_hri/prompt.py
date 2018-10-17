##! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from orion_hri.hri import HRI

from smach import State

class Prompt(smach.State):
    """
    Prompt

    """

    def __init__(self, question, valid_sentences, valid_objects_with_word, sentence_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'search_for_objects', 'ignore_last_object', 'aborted', 'preempted'],
                             input_keys=['arguments', 'objects', 'neg_objects'],
                             output_keys=['arguments', 'objects', 'neg_objects'])

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


        # (sentences,scores) = self.hri.prompt(self, self.question, valid_sentences, None, self.sentence_not_valid)
        (sentences,scores) = self.hri.prompt(self, self.question, valid_sentences, self.valid_objects_with_word, self.sentence_not_valid)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        #suggested_objects = []
        for sentence in sentences:
            if sentence == 'search for objects':
                return 'search_for_objects'
            elif sentence == 'ignore last object':
                userdata.neg_objects = []
                if len(userdata.objects) >= 1:
                    obj = userdata.objects[-1]
                    userdata.objects = userdata.objects[:-1]
                    self.hri.say("OK, I will ignore the " + obj)
                else:
                    self.hri.say("OK, but there are no more objects")
                return 'ignore_last_object'
            words = sentence.split(' ')

            print(sentence, words)

            if len(words) == 4: # bring me the OBJECT
                userdata.arguments.append(str(words[-1]))

                
                #rospy.loginfo('Suggested objects for %s: %s', str(words[-1]),  str(self.valid_objects_with_word[words[-1]]))
                #suggested_objects += self.valid_objects_with_word[words[-1]]
            elif len(words) == 5: # bring me the PROPERTY OBJECT
                userdata.arguments.append(str(words[-2] + ' ' + words[-1]))
                #rospy.loginfo('bSuggested objects for %s: %s', str(words[-2]),  str(self.valid_objects_with_word[words[-2]]))
                #rospy.loginfo('Suggested objects for %s: %s', str(words[-1]),  str(self.valid_objects_with_word[words[-1]]))
                #suggested_objects += self.valid_objects_with_word[words[-2]]
                #suggested_objects += self.valid_objects_with_word[words[-1]]
            else:
                rospy.logerr('Invalid sentence: %s', sentence)
            
            #self.hri.say("I understood: " +  sentence)

        # for obj in suggested_objects:
        #     if obj not in userdata.arguments:
        #         userdata.arguments.append(obj)
        # rospy.loginfo("All objects (incl. suggestions): %s", str(userdata.arguments))
        return 'succeeded'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        rospy.logwarn("Preempted!")
        State.request_preempt(self)



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

        (sentences,scores) = self.hri.prompt(self, self.question, self.possible_inputs, None, self.input_not_valid)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        
        #for sentence in sentences:            
        #    self.hri.say("I understood: " +  sentence)

        # most likely input
        userdata.input = sentences[0]
        userdata.arguments = []
        userdata.arguments.append(sentences[0])
            
        return 'succeeded'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        rospy.logwarn("Preempted!")
        State.request_preempt(self)
