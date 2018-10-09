#! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from tmc_rosjulius_msgs.msg import RecognitionResult

class Listen(smach.State):
    """
    Listen

    """

    def __init__(self, question, valid_sentences):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted'],
                             input_keys=[],
                             output_keys=['argument'])

        self.question = question
        self.valid_sentences = valid_sentences
        self.speaker=actionlib.SimpleActionClient('talk_request_action', TalkRequestAction)
        rospy.loginfo("Waiting for talk_request_action...")
        self.speaker.wait_for_server()
        rospy.loginfo("Speech action started")


        self.text_topic = '/hsrb/voice/text'
        #rospy.loginfo("Waiting for topic: /hsrb/voice/text")
        #rospy.Subscriber("/hsrb/voice/text", RecognitionResult  , self.callback)
        self.rate = rospy.Rate(10)
        
    def say(self, speech):
        print speech
        talk_goal = TalkRequestGoal()
        talk_goal.data.language = Voice.kEnglish
        talk_goal.data.sentence = speech
        self.speaker.send_goal(talk_goal)
        self.speaker.wait_for_result()
        
    def callback(self, data):

        sentences = []
        scores = []

        #print("VALID: " + str(self.valid_sentences))
        #print("DATA: " + str(data.sentences))        
        
        # filter out invalid sentences
        for i in range(len(data.sentences)):
            if data.sentences[i] in self.valid_sentences:
                sentences.append(data.sentences[i])
                scores.append(data.scores[i])
            else:
                rospy.loginfo("CMD IGNORE: " +  str(data.sentences[i]))

        if sentences:
            self.sentence = sentences[0] 
            self.score = scores[0]
            rospy.loginfo("SENTENCE: " + self.sentence + " , SCORE: " +  str(self.score))
        
    def execute(self, userdata):
        rospy.loginfo("Listen")
        self.sentence = None

        while self.sentence == None:
            self.say(self.question)
            data = RecognitionResult()
            try:
                rospy.loginfo('Waiting for speech input: %s', self.text_topic)
                data = rospy.wait_for_message(self.text_topic, RecognitionResult , timeout=30.0)
                rospy.loginfo('Got speech input') 
            except rospy.ROSException, e:
                rospy.logwarn("Failed to get speech input from %s" % self.text_topic)

            sentences = []
            scores = []

            for i in range(len(data.sentences)):
                if data.sentences[i] in self.valid_sentences:
                    sentences.append(data.sentences[i])
                    scores.append(data.scores[i])
                else:
                    rospy.loginfo("CMD IGNORE: " +  str(data.sentences[i]))
                    #

            if sentences:
                self.sentence = sentences[0] 
                self.score = scores[0]
                rospy.loginfo("SENTENCE: " + self.sentence + " , SCORE: " +  str(self.score))
            else:
                self.say("I didn't quite get that. Can you please say that again?")

        userdata.argument = self.sentence.split(' ')[-1]
        self.say("I understood: " +  self.sentence)
        
        return 'succeeded'