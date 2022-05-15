#! /usr/bin/env python3
import roslib; roslib.load_manifest('orion_hri')
import rospy
import time

import threading

import actionlib
import smach_ros

from  orion_hri.msg import *
from  orion_hri.wait_for_input_sm import WaitForInputSM

class WaitForInputActionServer:


    _feedback = orion_hri.msg.WaitForInputFeedback()
    _result   = orion_hri.msg.WaitForInputResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                orion_hri.msg.WaitForInputAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server: WaitForInput')


    def execute_cb(self, goal):

        rospy.loginfo('Received request...')

        # helper variables
        r = rospy.Rate(1)
        success = True

        
        # create the state machine
        sm = WaitForInputSM(goal.question, goal.possible_inputs, goal.timeout)

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
            self._feedback = orion_hri.msg.WaitForInputFeedback()
            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.loginfo('User input: %s' % str(sm.userdata.input))
            self._result = orion_hri.msg.WaitForInputResult()
            self._result.input = sm.userdata.input
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            #self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('wait_for_input_server')
    my_server = WaitForInputActionServer('wait_for_input')
    rospy.spin()
