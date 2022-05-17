#! /usr/bin/env python3

import rospy
from actionlib import SimpleActionServer, SimpleActionClient

from orion_actions.msg import HotwordListenAction, HotwordListenGoal, HotwordListenResult
from orion_actions.msg import SpeakAndListenAction, SpeakAndListenGoal, SpeakAndListenFeedback, SpeakAndListenResult
from std_srvs.srv import Empty

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
import speech_recognition as sr
from recogniser import ASR
import time, os
import numpy as np
from recorder import Recorder
# NOTE: No longer able to use Snowboy, discontinued
#from hotword import HotwordDetector
from text_classifier import parse_candidates


class SpeechServer(object):
    # create messages that are used to publish feedback/result
    _hotword_result = HotwordListenResult()

    def __init__(self, name):
        self._action_name = name

        # Recorder
        self.recorder = Recorder(save_audio=True)

        # ROS services
        self._rec_start_srv = rospy.Service('recording_start', Empty, self.start_recording)
        self._rec_stop_srv = rospy.Service('recording_stop', Empty, self.stop_recording)

        # ROS action servers
        self.snl_as = SimpleActionServer("speak_and_listen", SpeakAndListenAction, execute_cb=self.speak_and_listen_cb, auto_start=False)
        self.snl_as.start()
        # self._hotword_as = SimpleActionServer("hotword_listen", HotwordListenAction, execute_cb=self.hotword_listen_cb, auto_start=False)
        # self._hotword_as.start()

        # ROS publisher
        self.speech_text_pub = rospy.Publisher('speech_text', SpeechText, queue_size=10)

        # ROS action clients
        self.text_to_speech = SimpleActionClient('talk_request_action', TalkRequestAction)
        self.text_to_speech.wait_for_server(timeout=rospy.Duration(5))

        rospy.logwarn("SpeechServer started:")

    def start_recording(self):
        rospy.logwarn("Starting speech recording")
        self.recorder.start()

    def stop_recording(self):
        rospy.logwarn("Stopping speech recording")
        self.recorder.stop()

    def speak(self, text):
        talk_goal = TalkRequestGoal()
        talk_goal.data.language = Voice.kEnglish
        talk_goal.data.sentence = text
        self.text_to_speech.send_goal(talk_goal)
        self.text_to_speech.wait_for_result()
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

        parsed_candidates, candidate_params = parse_candidates(candidates, params)
        self.recorder.start()
        rospy.logwarn("Recording started")


        with Recorder() as rec:
            source = rec.frames_generator()

            rospy.loginfo("Started recording...")

            timelimit = time.time() + timeout if timeout else np.inf

            answer, param, confidence, transcription, succeeded = "", "", 0.0, "", False

            while timelimit - time.time() > 0:

                if self.snl_as.is_preempt_requested():
                    rospy.logwarn('%s: Preempted' % self._action_name)
                    self.snl_as.set_preempted()
                    return

                answer, param, transcription, confidence = asr.record(source, rec.config)
                rospy.logwarn('Answer: %s, Confidence: %s' % (answer, confidence))
                if confidence > 0.6:
                    break
                else:
                    snl_feedback = SpeakAndListenFeedback()
                    snl_feedback.answer, snl_feedback.param, snl_feedback.confidence, snl_feedback.transcription = answer, param, confidence, transcription
                    snl_feedback.remaining = timelimit - time.time() if timeout else 0
                    self.snl_as.publish_feedback(snl_feedback)
        if succeeded:
            self.speak("OK. You said " + answer)
        else:
            self.speak("Sorry, I didn't get it.")

        snl_result = SpeakAndListenResult()
        snl_result.answer, snl_result.param, snl_result.confidence, snl_result.transcription, snl_result.succeeded = answer, param, confidence, transcription, succeeded
        self.snl_as.set_succeeded(snl_result)

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
