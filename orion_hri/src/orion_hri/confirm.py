#! /usr/bin/env python3
import rospy
import smach
import smach_ros

import actionlib

from orion_hri.hri import HRI

from smach import State

class ConfirmInput(smach.State):
    """
    ConfirmInput

    """

    def __init__(self, question, positive_ex, negative_ex, answer_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_confirmed', 'aborted', 'preempted'],
                             input_keys=['arguments','objects', 'neg_objects'],
                             output_keys=['arguments','objects', 'neg_objects'])

        self.question = question
        self.positive_ex = positive_ex
        self.negative_ex = negative_ex
        self.answer_not_valid = answer_not_valid
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("STATE - ConfirmInput")

        self.hri.say('Please confirm what you said.')
        
        for arg in userdata.arguments:

            if arg not in userdata.objects and arg not in userdata.neg_objects:

                is_confirmed = self.hri.confirm(self, self.question  + str(arg) + '?', self.positive_ex, self.negative_ex, self.answer_not_valid)

                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'

                
                if is_confirmed:
                    if arg not in userdata.objects:
                        userdata.objects.append(arg)
                        if arg in ['tidy up', 'search for objects','ignore last object', 'move to start']:
                            self.hri.say('OK, I will ' + str(arg))
                        elif arg in ['bring me objects', 'find me objects']:
                            self.hri.say('OK, I will bring you objects')
                        else:
                            self.hri.say('OK, input confirmed: ' + str(arg))

                    return 'succeeded'
                else:
                    if arg not in userdata.neg_objects:
                        userdata.neg_objects.append(arg)
                    
        return 'not_confirmed'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        rospy.logwarn("Preempted!")
        State.request_preempt(self)



class Confirm(smach.State):
    """
    Confirm

    """

    def __init__(self, question, positive_ex, negative_ex, answer_not_valid=''):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_confirmed', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=['is_confirmed'])

        self.question = question
        self.positive_ex = positive_ex
        self.negative_ex = negative_ex
        self.answer_not_valid = answer_not_valid

        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("STATE - Confirm")


        is_confirmed = self.hri.confirm(self, self.question, self.positive_ex, self.negative_ex, self.answer_not_valid)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        
        userdata.is_confirmed = is_confirmed
        
        if is_confirmed:
            return 'succeeded'
        return 'not_confirmed'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        rospy.logwarn("Preempted!")
        State.request_preempt(self)

