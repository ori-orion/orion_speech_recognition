#! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from orion_hri.hri import HRI

class ConfirmInput(smach.State):
    """
    ConfirmInput

    """

    def __init__(self, question, positive_ex, negative_ex):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_confirmed', 'aborted', 'preempted'],
                             input_keys=['argument','objects'],
                             output_keys=['argument','objects'])

        self.question = question
        self.positive_ex = positive_ex
        self.negative_ex = negative_ex
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("Confirm")

        is_confirmed = self.hri.confirm(self.question  + str(userdata.argument) + '?', self.positive_ex, self.negative_ex)

        if is_confirmed:
            if userdata.argument not in userdata.objects:
                userdata.objects.append(userdata.argument)
            return 'succeeded'
        return 'not_confirmed'



class Confirm(smach.State):
    """
    Confirm

    """

    def __init__(self, question, positive_ex, negative_ex):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_confirmed', 'aborted', 'preempted'],
                             input_keys=['argument','objects'],
                             output_keys=['argument','objects'])

        self.question = question
        self.positive_ex = positive_ex
        self.negative_ex = negative_ex
        
        self.hri = HRI()

            
    def execute(self, userdata):
        rospy.loginfo("Confirm")

        is_confirmed = self.hri.confirm(self.question, self.positive_ex, self.negative_ex)

        if is_confirmed:
            if userdata.argument not in userdata.objects:
                userdata.objects.append(userdata.argument)
            return 'succeeded'
        return 'not_confirmed'

            
