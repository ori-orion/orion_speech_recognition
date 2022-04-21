#! /usr/bin/env python3

import rospy
from actionlib import SimpleActionServer, SimpleActionClient

from orion_actions.msg import HotwordListenAction, HotwordListenGoal, HotwordListenResult
from orion_actions.msg import SpeakAndListenAction, SpeakAndListenGoal, SpeakAndListenFeedback, SpeakAndListenResult

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
import speech_recognition as sr
# from wavenet.recognize import WaveNet
from recogniser import ASR
import time, os
import numpy as np
from record import Recorder
# NOTE: No longer able to use Snowboy, discontinued
#from hotword import HotwordDetector

# import ebbhrd_msgs.msg


class SpeechServer(object):
    # create messages that are used to publish feedback/result
    _snl_feedback = SpeakAndListenFeedback()
    _snl_result = SpeakAndListenResult()
    _hotword_result = HotwordListenResult()

    def __init__(self, name):
        self._action_name = name
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 0.5
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)

        self.output_text = SimpleActionClient('talk_request_action', TalkRequestAction)
        rospy.loginfo("Waiting for talk_request_action...")
        self.output_text.wait_for_server(timeout=rospy.Duration(5))
        rospy.loginfo("Speech action started")

        # self.wavenet = WaveNet()

        rospy.logwarn("SpeechServer started:")

        self._snl_as = SimpleActionServer("speak_and_listen", SpeakAndListenAction, execute_cb=self.speak_and_listen_cb, auto_start=False)
        self._snl_as.start()



        #self._hotword_as = SimpleActionServer("hotword_listen", HotwordListenAction, execute_cb=self.hotword_listen_cb, auto_start=False)
        #self._hotword_as.start()

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
        # rospy.logwarn("Candidates:")
        # rospy.logwarn(candidates)
        # rospy.logwarn("Params:")
        # rospy.logwarn(params)

        if question:
            self.speak(question)

        asr = ASR(self.recognizer, "self.wavenet")
        asr.set_candidates(candidates, params)

        # with sr.Microphone() as source:
        with Recorder() as rec:
            source = rec.frames_generator()

            rospy.loginfo("Started recording...")

            timelimit = time.time() + timeout if timeout else np.inf

            answer, param, confidence, transcription, succeeded = "", "", 0.0, "", False

            while timelimit - time.time() > 0:

                if self._snl_as.is_preempt_requested():
                    rospy.logwarn('%s: Preempted' % self._action_name)
                    self._snl_as.set_preempted()
                    return

                answer, param, confidence, transcription, succeeded = asr.record(source, rec.config)
                rospy.logwarn('Answer: %s, Confidence: %s' % (answer, confidence))
                if succeeded:
                    break
                else:
                    self._snl_feedback.answer, self._snl_feedback.param, self._snl_feedback.confidence, self._snl_feedback.transcription = answer, param, confidence, transcription
                    self._snl_feedback.remaining = timelimit - time.time() if timeout else 0
                    self._snl_as.publish_feedback(self._snl_feedback)

        if succeeded:
            self.speak("OK. You said " + answer[0])
            pass;
        else:
            self.speak("Sorry, I didn't get it.")

        self._snl_result.answer, self._snl_result.param, self._snl_result.confidence, self._snl_result.transcription, self._snl_result.succeeded = answer[0], param, confidence, transcription, succeeded
        self._snl_as.set_succeeded(self._snl_result)

    def hotword_listen_cb(self, goal):

        timeout = goal.timeout
        hotwords = goal.hotwords
        timelimit = time.time() + timeout if timeout else np.inf

        rospy.logwarn("HotwordListen action started:")

        def preempt_callback():
            if self._hotword_as.is_preempt_requested():
                rospy.logwarn('%s: Preempted' % self._action_name)
                self._hotword_as.set_preempted()
                return True
            if timelimit - time.time() <= 0:
                return True
            return False

        detector = HotwordDetector(hotwords, preempt_callback)
        detector.run()

        if detector.detected_hotword:
            rospy.loginfo("Identified hotword: %s" % detector.detected_hotword)
            self.speak("You said " + detector.detected_hotword)
        else:
            rospy.loginfo("No hotword detected")

        self._hotword_result.hotword, self._hotword_result.succeeded = detector.detected_hotword, detector.detected_hotword != ""
        self._hotword_as.set_succeeded(self._hotword_result)


if __name__ == '__main__':
    rospy.init_node('speech')
    server = SpeechServer(rospy.get_name())
    rospy.spin()
