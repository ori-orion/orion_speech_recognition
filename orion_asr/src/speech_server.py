#! /usr/bin/env python

import rospy
from actionlib import SimpleActionServer

from orion_actions.msg import SpeakAndListenAction, SpeakAndListenGoal, SpeakAndListenFeedback, SpeakAndListenResult

import speech_recognition as sr
# from wavenet.recognize import WaveNet
from recogniser import ASR
import time
import numpy as np

class SpeechServer(object):
    # create messages that are used to publish feedback/result
    _feedback = SpeakAndListenFeedback()
    _result = SpeakAndListenResult()

    def __init__(self, name):
        self._action_name = name
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 0.5
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # self.wavenet = WaveNet()
        
        rospy.logwarn("SpeechServer started:")

        self._as = SimpleActionServer("speak_and_listen", SpeakAndListenAction, execute_cb=self.speak_and_listen_cb, auto_start=False)
        self._as.start()

    def speak_and_listen_cb(self, goal):

        question = goal.question
        candidates = goal.candidates
        params = goal.params
        timeout = goal.timeout

        rospy.logwarn("SpeakAndListen action started:")
        rospy.logwarn("Question: " + question)
        rospy.logwarn("Candidates:")
        rospy.logwarn(candidates)
        rospy.logwarn("Params:")
        rospy.logwarn(params)

        if self._as.is_preempt_requested():
            rospy.logwarn('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return

        asr = ASR(self.recognizer, "self.wavenet")

        with sr.Microphone() as source:
            print("Started recording...")

            timelimit = time.time() + timeout if timeout else np.inf

            while timelimit - time.time() > 0:
                results = asr.main(source, candidates, params)
                rospy.logwarn('Answer: %s, Confidence: %s' % (results[0], results[2]))
                if results[-1]:
                    self._result.answer, self._result.param, self._result.confidence, self._result.transcription, self._result.succeeded = results
                    self._as.set_succeeded(self._result)
                    return
                else:
                    self._feedback.answer, self._feedback.param, self._feedback.confidence, self._feedback.transcription, _ = results
                    self._feedback.remaining = timelimit - time.time() if timeout else 0
                    self._as.publish_feedback(self._feedback)
            
            self.recognizer.adjust_for_ambient_noise(source)


if __name__ == '__main__':
    rospy.init_node('speech')
    server = SpeechServer(rospy.get_name())
    rospy.spin()