#! /usr/bin/env python3

import rospy
from actionlib import SimpleActionServer, SimpleActionClient

from orion_actions.msg import SpeakAndListenAction, SpeakAndListenGoal, SpeakAndListenFeedback, SpeakAndListenResult, \
    SpeechText, Hotword
from std_msgs.msg import Header, String
from std_srvs.srv import Empty

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
import time, os
import numpy as np

from hotword.porcupine import PorcupineHotwordDetector
from recorder import Recorder
from text_classifier import parse_candidates, classify_text


class SpeechServer:
    def __init__(self, name, confidence_thresh=0.6, classifier="levenshtein"):
        rospy.loginfo(f"Initialising {name}")
        self._action_name = name
        self.confidence_thresh = confidence_thresh
        self.classifier = classifier

        # Recorder
        self.recorder = Recorder(save_audio=True)

        # ROS services
        self._rec_start_srv = rospy.Service('recording_start', Empty, self.start_recording)
        self._rec_stop_srv = rospy.Service('recording_stop', Empty, self.stop_recording)

        # ROS action servers
        self.snl_as = SimpleActionServer("speak_and_listen", SpeakAndListenAction, execute_cb=self.speak_and_listen_cb, auto_start=False)
        self.snl_as.start()

        # ROS publisher
        self.speech_text_pub = rospy.Publisher('speech_text', SpeechText, queue_size=10)
        self.hotword_pub = rospy.Publisher('hotword', Hotword, queue_size=10)

        # ROS action clients
        self.text_to_speech = SimpleActionClient('talk_request_action', TalkRequestAction)
        self.text_to_speech.wait_for_server(timeout=rospy.Duration(5))

        rospy.loginfo("SpeechServer started:")

        self.detector = PorcupineHotwordDetector(keywords=("hey robot", "i'm ready"))
        self.detector.start(self._hotword_listen_cb)
        rospy.loginfo("Hotword detector started:")

        rospy.on_shutdown(self.detector.stop)

    def _hotword_listen_cb(self, hotword):
        rospy.loginfo(f"Detected hotword: {hotword}")
        hotword_msg = Hotword()
        hotword_msg.stamp = rospy.Time.now()
        hotword_msg.hotword = hotword
        self.hotword_pub.publish(hotword_msg)
        return False

    def _speech_to_text_cb(self, frames: list, transcriptions: dict):
        speech_text = SpeechText()
        speech_text.header = Header()
        speech_text.header.stamp = rospy.Time.now()
        speech_text.header.frame_id = "speech_text"
        speech_text.detectors = list(transcriptions.keys())
        speech_text.transcriptions = list(transcriptions.values())
        self.speech_text_pub.publish(speech_text)

    def start_recording(self):
        rospy.loginfo("Starting speech recording")
        self.recorder.start(self._speech_to_text_cb)

    def stop_recording(self):
        rospy.loginfo("Stopping speech recording")
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

        rospy.loginfo("SpeakAndListen action started:")
        rospy.loginfo("Question: " + question)
        # rospy.loginfo("Candidates:")
        # rospy.loginfo(candidates)
        # rospy.loginfo("Params:")
        # rospy.loginfo(params)

        if question:
            self.speak(question)

        start_time = time.time()
        timelimit = start_time + timeout if timeout else np.inf

        self.start_recording()
        rospy.loginfo("Recording started")

        parsed_candidates, candidate_params = parse_candidates(candidates, params)

        while not rospy.is_shutdown() and timelimit - time.time() > 0:
            results, timestamp = self.recorder.outputs_q.get()
            if timestamp < start_time:
                continue
            print(results, timestamp)

            if self.snl_as.is_preempt_requested():
                rospy.logwarn('Preempted SpeakAndListen')
                self.snl_as.set_preempted()
                return

            answer, param, transcription, confidence = classify_text(parsed_candidates, candidate_params, list(results.values()), algorithm=self.classifier)
            rospy.loginfo('Answer: %s, Confidence: %s' % (answer, confidence))

            if confidence > self.confidence_thresh:
                self.speak("OK. You said " + answer)
                snl_result = SpeakAndListenResult()
                snl_result.answer, snl_result.param, snl_result.confidence, snl_result.transcription, snl_result.succeeded = answer, param, confidence, transcription, True
                self.snl_as.set_succeeded(snl_result)
                break
            else:
                snl_feedback = SpeakAndListenFeedback()
                snl_feedback.answer, snl_feedback.param, snl_feedback.confidence, snl_feedback.transcription = answer, param, confidence, transcription
                snl_feedback.remaining = timelimit - time.time() if timeout else 0
                self.snl_as.publish_feedback(snl_feedback)
        else:   # if while loop exits without a break, i.e. no success
            rospy.logwarn('Aborted SpeakAndListen')
            self.speak("Sorry, I didn't get it.")
            self.snl_as.set_aborted()
        self.stop_recording()


if __name__ == '__main__':
    rospy.init_node('speech')
    server = SpeechServer(rospy.get_name())
    rospy.spin()
