#! /usr/bin/env python
import roslib; roslib.load_manifest('orion_hri')
import rospy
import time

import threading

import actionlib
import smach_ros

from  orion_hri.msg import *
from  orion_hri.wait_for_confirmation_sm import WaitForConfirmationSM

class WaitForConfirmationActionServer:


    _feedback = orion_hri.msg.WaitForConfirmationFeedback()
    _result   = orion_hri.msg.WaitForConfirmationResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                orion_hri.msg.WaitForConfirmationAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server: WaitForConfirmation')


    def execute_cb(self, goal):

        rospy.loginfo('Received request...')

        # helper variables
        r = rospy.Rate(1)
        success = True


        # TODO: currently these are strings, but rather should be lists...(however MDP can't handle it easily)
        positive_answers = [goal.positive_answers]
        negative_answers = [goal.negative_answers]
        
        # create the state machine
        sm = WaitForConfirmationSM(goal.question, positive_answers, negative_answers, goal.timeout)

        sis = smach_ros.IntrospectionServer(self._action_name, sm, '/SM_ROOT')
        sis.start()

        


        smach_thread = threading.Thread(target = sm.execute)
        smach_thread.start()

        #outcome = self.agent.execute_sm_with_introspection()
        r.sleep()

        while sm.is_running() and not sm.preempt_requested():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                sm.request_preempt()
                success = False
                break

            # get current pose form state machine
            self._feedback = orion_hri.msg.WaitForConfirmationFeedback()
            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.loginfo('Is confirmed: %s' % str(sm.userdata.is_confirmed))
            self._result = orion_hri.msg.WaitForConfirmationResult()
            self._result.is_confirmed = sm.userdata.is_confirmed
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('wait_for_confirmation_server')
    my_server = WaitForConfirmationActionServer('wait_for_confirmation')
    rospy.spin()
