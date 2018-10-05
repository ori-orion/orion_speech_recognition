#! /usr/bin/env python
import rospy
import smach
import smach_ros

import actionlib

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from tmc_rosjulius_msgs.msg import RecognitionResult

class Confirm(smach.State):
    """
    Confirm

    """

    def __init__(self, question, only_question, confirmation_sentences, rejection_sentences):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'not_confirmed', 'aborted', 'preempted'],
                             input_keys=['argument', 'objects'],
                             output_keys=['argument', 'objects'])

        self.question = question
        self.only_question = only_question
        self.confirmation_sentences = confirmation_sentences
        self.rejection_sentences = rejection_sentences
        self.isConfirmed = False
        self.isRejected = False
        
        self.speaker=actionlib.SimpleActionClient('talk_request_action', TalkRequestAction)
        rospy.loginfo("Waiting for talk_request_action...")
        self.speaker.wait_for_server()
        rospy.loginfo("Speech action started")

        rospy.loginfo("Waiting for topic: /hsrb/voice/text")
        rospy.Subscriber("/hsrb/voice/text", RecognitionResult  , self.callback)

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

        self.isConfirmed = False
        self.isRejected = False      
        # filter out invalid sentences
        for i in range(len(data.sentences)):
            if data.sentences[i] in self.confirmation_sentences: 
                sentences.append(data.sentences[i])
                scores.append(data.scores[i])
                self.isConfirmed = True
            elif data.sentences[i] in self.rejection_sentences:
                sentences.append(data.sentences[i])
                scores.append(data.scores[i])
                self.isRejected = True
            else:
                rospy.loginfo("CONFIRM IGNORE: " +  str(data.sentences[i]))

        if sentences:
            self.sentence = sentences[0] 
            self.score = scores[0]
            rospy.loginfo("SENTENCE: " + self.sentence + " , SCORE: " +  str(self.score))
        
    def execute(self, userdata):
        rospy.loginfo("Listen")

        self.isConfirmed = False
        self.isRejected = False      

        self.sentence = None
        if self.only_question:
            self.say(self.question)
        else:
            self.say(self.question + str(userdata.argument) + '?')

        rospy.loginfo("Waiting for speech input...")
        while self.sentence == None:
            self.rate.sleep()

            
        self.say("I understood: " +  self.sentence)

        if self.isConfirmed:
            if userdata.argument not in userdata.objects:
                userdata.objects.append(userdata.argument)
            return 'succeeded'
        elif self.isRejected:
            return 'not_confirmed'
        else: # should never get here...
            rospy.logerror('Something went wrong...')
            return 'aborted'
