#! /usr/bin/env python

import rospy
import time
import actionlib

from arm_robot.msg import CountAction, CountGoal, CountResult

def do_count( goal):
    i = goal.count
    while i > 0:
        i -= 1
        rospy.sleep( 1)

    result = CountResult()
    result.count = i
#    result.updates_sent = 0
    server.set_succeeded( result)

rospy.init_node('count_action_server')
server = actionlib.SimpleActionServer( 'count', CountAction, do_count, False)
server.start()

rospy.spin()
