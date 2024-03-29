#! /usr/bin/env python3
import rospy

import actionlib
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from tmc_rosjulius_msgs.msg import RecognitionResult


class HRI():

    def __init__(self):

        self.input_text = '/hsrb/voice/text' 
        rospy.loginfo("Input text: %s", self.input_text)
        
        self.output_text = actionlib.SimpleActionClient('talk_request_action', TalkRequestAction)
        rospy.loginfo("Waiting for talk_request_action...")
        self.output_text.wait_for_server()
        rospy.loginfo("Speech action started")

        self.colors = { 'say' : 'yellow',
                        'alert': 'red',
                        'prompt' : 'blue',
                        'confirm' : 'cyan'}
        

    def _get_input_text(self, timeout):

        data = RecognitionResult()
        try:
            rospy.loginfo('Waiting for speech input: %s', self.input_text)
            data = rospy.wait_for_message(self.input_text, RecognitionResult , timeout)
            rospy.loginfo('Got speech input') 
        except rospy.ROSException:
            rospy.logwarn("Failed to get speech input from %s" % self.input_text)
        return data


    def _filter_input_text(self, recognition_result, examples, keywords=None):

        sentences = []
        scores = []

        suggested_sentences = []
        suggested_scores = []
        prefix = 'bring me the'
        added_keywords = []
        for i in range(len(recognition_result.sentences)):
            rospy.loginfo("Speech recognition (unfiltered): %s - %s", recognition_result.sentences[i], str(recognition_result.scores[i]))
            if recognition_result.sentences[i] in examples:
                sentences.append(recognition_result.sentences[i])
                scores.append(recognition_result.scores[i])

            # make suggestions
            if keywords:
                for k in keywords:
                    if k not in added_keywords:
                        if k in recognition_result.sentences[i] and recognition_result.sentences[i] not in ['yes please', 'no cancel', 'no thanks', 'okay done', 'yes correct']: 
                            for val in keywords[k]:
                                s = prefix + ' ' + val
                                if s not in sentences:
                                    rospy.loginfo('Suggested object for %s: %s', str(k),  str(val))
                                    suggested_sentences.append(s)
                                    suggested_scores.append(0.0)
                                    added_keywords.append(k)


        for i in range(len(suggested_sentences)):
            if suggested_sentences[i] not in sentences:
                sentences.append(suggested_sentences[i])
                scores.append(suggested_scores[i])

        return (sentences, scores)
    
        
    def tts(self, text, timeout, color):

        talk_goal = TalkRequestGoal()
        talk_goal.data.language = Voice.kEnglish
        talk_goal.data.sentence = text
        self.output_text.send_goal(talk_goal)        
        #self.output_text.wait_for_result(rospy.Duration(timeout))
        self.output_text.wait_for_result()
        rospy.loginfo('SPEECH OUTPUT: %s', text)
        rospy.sleep(1.0)
        # TODO: set color of robot

    def say(self, text, timeout=60):
        self.tts(text, timeout, self.colors['say'])
    
    def alert(self, text, timeout=60):
        self.tts(text, timeout, self.colors['alert'])
        

    def prompt(self, state, text, positive_ex, keywords=None, text_not_valid='', repeat_after_sec=15, timeout=60, color='white'):
        sentences = []
        scores = []
        
        while not sentences and not  state.preempt_requested():
            self.say(text, timeout)

            rospy.loginfo("positive_ex: %s", str(positive_ex))

            result = self._get_input_text(repeat_after_sec)

            if state.preempt_requested():
                return (sentences, scores)

            (sentences, scores) = self._filter_input_text(result, positive_ex, keywords)

            if not sentences:
                self.say(text_not_valid, timeout)

        rospy.loginfo("SENTENCES: " + str(sentences) + " , SCORE: " +  str(scores))
        return (sentences, scores)
    
    def confirm(self, state, text, positive_ex, negative_ex, text_not_valid='', repeat_after_sec=15, timeout=60, color='white'):

        is_confirmed = False

        sentences = []
        scores = []
        
        self.sentence = ''
        self.score = 0.0
        while self.sentence == ''  and not  state.preempt_requested():
            self.say(text, timeout)

            rospy.loginfo("positive_ex: %s, negative: %s", str(positive_ex), str(negative_ex))
            
            
            result = self._get_input_text(repeat_after_sec)

            if state.preempt_requested():
                return (sentences, scores)
            
            (sentences, scores) = self._filter_input_text(result, positive_ex + negative_ex)

            if sentences:
                # select text with highest probability (first in the list)
                self.sentence = sentences[0] 
                self.score = scores[0]

                if self.sentence in positive_ex:
                    is_confirmed = True
                elif self.sentence in negative_ex:
                    is_confirmed = False
                else:
                    rospy.logerr("Invalid text input!")
            else:
                self.say(text_not_valid, timeout)

        rospy.loginfo("SENTENCE: " + self.sentence + "  SCORE: " +  str(self.score) + ' , IS_CONFIRMED:' + str(is_confirmed))
        return is_confirmed

    
    
            
            

    
