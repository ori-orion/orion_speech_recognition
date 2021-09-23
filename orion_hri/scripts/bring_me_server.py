#! /usr/bin/env python3
import roslib; roslib.load_manifest('orion_hri')
import rospy
import time

import threading

import actionlib
import smach_ros

from  orion_hri.msg import *
from  orion_hri.bring_me_sm import BringMeSM

_EPSILON = 0.0001

class WaitForInstructionActionServer:


    _feedback = orion_hri.msg.WaitForInstructionFeedback()
    _result   = orion_hri.msg.WaitForInstructionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                orion_hri.msg.WaitForInstructionAction,
                                                execute_cb=self.execute_cb,
                                                auto_start = False)

        self._as.start()
        rospy.loginfo('Started action server: WaitForInstruction')


    def execute_cb(self, goal):

        rospy.loginfo('Received request...')

        # helper variables
        r = rospy.Rate(1)
        success = True



        
        # create the state machine
        sm = BringMeSM()

        sis = smach_ros.IntrospectionServer('wait_for_instruction', sm, '/SM_ROOT')
        sis.start()

        
        # sm.userdata.x  

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

            #rospy.loginfo(self.agent.get_sm().get_active_states())
            userdata = sm.userdata

            # get current pose form state machine
            self._feedback = orion_hri.msg.WaitForInstructionFeedback()
            self._as.publish_feedback(self._feedback)

            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.loginfo('Objects: %s' % sm.userdata.objects)
            self._result = orion_hri.msg.WaitForInstructionResult()
            self._result.objects = []
            for obj in sm.userdata.objects:
                o = obj.replace(' ','_')
                self._result.objects.append(o)
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            #self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('bring_me_server')
    my_server = WaitForInstructionActionServer('wait_for_instruction')
    rospy.spin()
