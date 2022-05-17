#!/usr/bin/env python3
import rospy
from orion_asr.msg import SpeechText
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.transcriptions)


def listener():
    rospy.init_node('speech_text_listener', anonymous=True)
    rospy.Subscriber("speech_text", SpeechText, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
