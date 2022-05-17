#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty


def recording_stop_client():
    rospy.wait_for_service('recording_stop')
    try:
        speech_stop = rospy.ServiceProxy('recording_stop', Empty)
        speech_stop()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    recording_stop_client()
