#! /usr/bin/env python3

import rospy
import actionlib

from arm_robot.msg import CountAction, CountGoal, CountResult

rospy.init_node( 'count_action_client')
client = actionlib.SimpleActionClient( 'count', CountAction)
client.wait_for_server()

goal = CountGoal()
goal.count = 10

client.send_goal(goal)
client.wait_for_result()

print('count = %d' % client.get_result().count)
