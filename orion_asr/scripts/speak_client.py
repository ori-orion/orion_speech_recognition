#!/usr/bin/env python3

import sys
import rospy
from actionlib import SimpleActionClient
from tmc_msgs.msg import TalkRequestGoal, Voice, TalkRequestAction


def recording_start_client(text):
    text_to_speech = SimpleActionClient('talk_request_action', TalkRequestAction)
    text_to_speech.wait_for_server(timeout=rospy.Duration(5))

    talk_goal = TalkRequestGoal()
    talk_goal.data.language = Voice.kEnglish
    talk_goal.data.sentence = text

    text_to_speech.send_goal(talk_goal)
    text_to_speech.wait_for_result()
    rospy.loginfo('SPEECH OUTPUT: %s', text)


if __name__ == "__main__":
    recording_start_client(sys.argv[-1])
