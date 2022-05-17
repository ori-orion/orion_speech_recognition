#!/usr/bin/env python3

import sys
import rospy
from std_srvs.srv import Empty


def recording_start_client():
    rospy.wait_for_service('recording_start')
    try:
        speech_start = rospy.ServiceProxy('recording_start', Empty)
        speech_start()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    recording_start_client()
