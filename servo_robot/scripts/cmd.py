#!/usr/bin/env python

import sys
import time
import math

import rospy
import roslib

import moveit_commander

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose

NODE_NAME = 'cmd'
JOINT_NAME_ARRAY = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

if __name__ == '__main__':
    moveit_commander.roscpp_initialize( sys.argv)
    rospy.init_node( NODE_NAME, anonymous=False)

    msg_cmd = JointTrajectory()
    msg_cmd.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    msg_cmd.points.append( JointTrajectoryPoint())
    msg_cmd.points[0].time_from_start.secs = 1.0
    msg_cmd.points[0].time_from_start.nsecs = 0.0

    pub_cmd = rospy.Publisher( '/arm_controller/command', JointTrajectory, queue_size=10)

    msg_cmd.header.stamp = rospy.Time.now()
    msg_cmd.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_cmd.publish( msg_cmd)

    rospy.sleep( 2)

    msg_cmd.header.stamp = rospy.Time.now()
    msg_cmd.points[0].positions = [0.1, -0.5, 0.5, 0.75, 0.0, 0.0]
    pub_cmd.publish( msg_cmd)

    rospy.sleep( 2)

    msg_cmd.header.stamp = rospy.Time.now()
    msg_cmd.points[0].positions = [1.5, -1.5, -0.5, -0.75, 0.0, 0.0]
    pub_cmd.publish( msg_cmd)

    rospy.sleep( 2)

    msg_cmd.header.stamp = rospy.Time.now()
    msg_cmd.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_cmd.publish( msg_cmd)

    moveit_commander.roscpp_shutdown()
