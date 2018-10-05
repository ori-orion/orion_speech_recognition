#! /usr/bin/env python
import rospy
import smach
import smach_ros


class RecognizeSpeech(smach.State):
    """
    RecognizeSpeech

    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo("Recognize speech")
        return 'succeeded'
