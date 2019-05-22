#! /usr/bin/env python
import rospy
import smach
import smach_ros

from orion_hri.hri import HRI

class Shutdown(smach.State):
    """
    Shutdown

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=['objects'],
                             output_keys=[])

        self.hri = HRI()

    def execute(self, userdata):
        rospy.loginfo("STATE - Shutdown")

        rate = rospy.Rate(2)
        
        if userdata.objects:
            self.hri.say("I will bring you the following objects:")
            for obj in userdata.objects:
                self.hri.say(obj)
                rate.sleep()
                
        return 'succeeded'
