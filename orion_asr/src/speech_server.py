#! /usr/bin/env python

import rospy
from actionlib import SimpleActionServer, SimpleActionClient

from orion_actions.msg import HotwordListenAction, HotwordListenGoal, HotwordListenResult
from orion_actions.msg import SpeakAndListenAction, SpeakAndListenGoal, SpeakAndListenFeedback, SpeakAndListenResult

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
import speech_recognition as sr
# from wavenet.recognize import WaveNet
from recogniser import ASR
import time
import numpy as np

EXIT_WORDS = ["stop", "cancel", "finish", "exit", "terminate", "quit"]


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

        self.output_text = SimpleActionClient('talk_request_action', TalkRequestAction)
        rospy.loginfo("Waiting for talk_request_action...")
        self.output_text.wait_for_server()
        rospy.loginfo("Speech action started")

        # self.wavenet = WaveNet()
        
        rospy.logwarn("SpeechServer started:")

        self._as = SimpleActionServer("speak_and_listen", SpeakAndListenAction, execute_cb=self.speak_and_listen_cb, auto_start=False)
        self._as.start()

        self._hotword_as = SimpleActionServer("hotword_listen", HotwordListenAction, execute_cb=self.hotword_listen_cb, auto_start=False)
        self._hotword_as.start()

    def speak(self, text):

        talk_goal = TalkRequestGoal()
        talk_goal.data.language = Voice.kEnglish
        talk_goal.data.sentence = text
        self.output_text.send_goal(talk_goal)
        self.output_text.wait_for_result()
        rospy.loginfo('SPEECH OUTPUT: %s', text)

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

        if question:
            self.speak(question)

        asr = ASR(self.recognizer, "self.wavenet")

        with sr.Microphone() as source:
            print("Started recording...")

            timelimit = time.time() + timeout if timeout else np.inf

            answer, param, confidence, transcription, succeeded = "", "", 0.0, "", False

            while timelimit - time.time() > 0:
                answer, param, confidence, transcription, succeeded = asr.main(source, candidates, params)
                rospy.logwarn('Answer: %s, Confidence: %s' % (answer, confidence))
                if succeeded:
                    break
                else:
                    self._feedback.answer, self._feedback.param, self._feedback.confidence, self._feedback.transcription = answer, param, confidence, transcription
                    self._feedback.remaining = timelimit - time.time() if timeout else 0
                    self._as.publish_feedback(self._feedback)

            if succeeded:
                self.speak("OK. You said " + answer)
            else:
                self.speak("Sorry, I didn't get it.")
                self.recognizer.adjust_for_ambient_noise(source)

            self._result.answer, self._result.param, self._result.confidence, self._result.transcription, self._result.succeeded = answer, param, confidence, transcription, succeeded
            self._as.set_succeeded(self._result)

    def hotword_listen_cb(self, goal):

        timeout = goal.timeout

        rospy.logwarn("HotwordListen action started:")

        if self._hotword_as.is_preempt_requested():
            rospy.logwarn('%s: Preempted' % self._action_name)
            self._hotword_as.set_preempted()
            return

        asr = ASR(self.recognizer, "self.wavenet")

        with sr.Microphone() as source:
            print("Started recording...")

            timelimit = time.time() + timeout if timeout else np.inf

            answer, confidence, succeeded = "", 0.0, False

            while timelimit - time.time() > 0:
                answer, _, confidence, _, succeeded = asr.main(source, EXIT_WORDS, [])
                rospy.logwarn('Answer: %s, Confidence: %s' % (answer, confidence))
                if succeeded:
                    break
                else:
                    self._feedback.confidence = confidence
                    self._feedback.remaining = timelimit - time.time() if timeout else 0
                    self._hotword_as.publish_feedback(self._feedback)

            if succeeded:
                self.speak("OK. You said " + answer)
            else:
                self.speak("Sorry, I didn't get it.")
                self.recognizer.adjust_for_ambient_noise(source)

            self._result.confidence, self._result.succeeded = confidence, succeeded
            self._hotword_as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('speech')
    server = SpeechServer(rospy.get_name())
    rospy.spin()