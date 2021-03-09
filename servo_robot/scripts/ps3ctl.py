#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import sys
import time
import math
import copy
import signal
import threading
import datetime

import rospy
import rosbag
import sensor_msgs

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension

from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from arm_robot.msg import DataList

import pport
import soem
from soem import CTRL_RELATE, CTRL_IMMEDIATLY, FACTOR_GRAD, HOME_CURRENT_POSITION

TYPE_LED = JoyFeedback.TYPE_LED
TYPE_RUMBLE = JoyFeedback.TYPE_RUMBLE
TYPE_BUZZER = JoyFeedback.TYPE_BUZZER

PS3_BUTTON_SELECT          = 0
PS3_BUTTON_STICK_LEFT      = 1
PS3_BUTTON_STICK_RIGHT     = 2
PS3_BUTTON_START           = 3
PS3_BUTTON_CROSS_UP        = 4
PS3_BUTTON_CROSS_RIGHT     = 5
PS3_BUTTON_CROSS_DOWN      = 6
PS3_BUTTON_CROSS_LEFT      = 7
PS3_BUTTON_REAR_LEFT_2     = 8
PS3_BUTTON_REAR_RIGHT_2    = 9
PS3_BUTTON_REAR_LEFT_1     = 10
PS3_BUTTON_REAR_RIGHT_1    = 11
PS3_BUTTON_ACTION_TRIANGLE = 12
PS3_BUTTON_ACTION_CIRCLE   = 13
PS3_BUTTON_ACTION_CROSS    = 14
PS3_BUTTON_ACTION_SQUARE   = 15
PS3_BUTTON_PAIRING         = 16

PS3_AXIS_STICK_LEFT_LEFTWARDS   = 0
PS3_AXIS_STICK_LEFT_UPWARDS     = 1
PS3_AXIS_STICK_RIGHT_LEFTWARDS  = 2
PS3_AXIS_STICK_RIGHT_UPWARDS    = 3
PS3_AXIS_BUTTON_CROSS_UP        = 4
PS3_AXIS_BUTTON_CROSS_RIGHT     = 5
PS3_AXIS_BUTTON_CROSS_DOWN      = 6
PS3_AXIS_BUTTON_CROSS_LEFT      = 7
PS3_AXIS_BUTTON_REAR_LEFT_2     = 8
PS3_AXIS_BUTTON_REAR_RIGHT_2    = 9
PS3_AXIS_BUTTON_REAR_LEFT_1     = 10
PS3_AXIS_BUTTON_REAR_RIGHT_1    = 11
PS3_AXIS_BUTTON_ACTION_TRIANGLE = 12
PS3_AXIS_BUTTON_ACTION_CIRCLE   = 13
PS3_AXIS_BUTTON_ACTION_CROSS    = 14
PS3_AXIS_BUTTON_ACTION_SQUARE   = 15
PS3_AXIS_ACCELEROMETER_LEFT     = 16
PS3_AXIS_ACCELEROMETER_FORWARD  = 17
PS3_AXIS_ACCELEROMETER_UP       = 18
PS3_AXIS_GYRO_YAW               = 19

PS3_LED_1 = 0
PS3_LED_2 = 1
PS3_LED_3 = 2
PS3_LED_4 = 3

PS3_RUMBLE_1 = 0
PS3_RUMBLE_2 = 1

X = 0
Y = 1
Z = 2
ROLL = 3
PITCH = 4
YAW = 5

# Status
ROSBAG = 10 # Tag
ROSBAG_NONE = 0
ROSBAG_WRITE = 1
ROSBAG_READ = 2

NODE_NAME = 'ps3ctl'

CONTROLLER_COMMAND = '/arm_controller/command'
#TOPIC_JOINT_STATES = '~joint_states'
JOINT_STATES = 'joint_states'
JOY_DATA = 'joy/data'
#JOY_RATE = 'ps3joy_node/autorepeat_rate'
SMART_JOINT_STATES = 'smart/joint_states'
SMART_TRANSFORMS = 'smart/tf'
SMART_STATUS = 'smart/status'

DEFAULT_FREQUENCY = 10 # Hz
JOINT_NAME_ARRAY = ['a1_joint', 'a2_joint', 'a3_joint', 'a4_joint', 'a5_joint', 'a6_joint']

SPEED_MAX = 1 # m/s

STOP_MAGIC = 999999.99

def create_fname( directory, ext):
    d = datetime.date.today()
    pattern = re.compile( r'^%4d%02d%02d-(\d+)\.%s$' % ( d.year, d.month, d.day, ext))
    files = os.listdir( directory)
    files = filter( lambda x: pattern.match( x), files)

    if not files:
        return "%s/%4d%02d%02d-1.%s" % ( directory, d.year, d.month, d.day, ext)

    nmax = 0
    for name in files:
        r = pattern.match( name)
        n = int( r.group( 1))
        if n > nmax:
            nmax = n

    return "%s/%4d%02d%02d-%d.%s" % ( directory, d.year, d.month, d.day, nmax + 1, ext)

def radian( rad):
    return int( round( math.degrees( rad), 1) * 10)


class Listener( threading.Thread):
    def __init__( self):
#        threading.Thread.__init__( self)
        super( Listener, self).__init__()
        self.daemon = True
        self.done = True
        self.work = False
        self.stop = False
        self.doing = False
        self.motors = [ False] * 6
        self.pins = ( pport.PIN2, pport.PIN3, pport.PIN4, pport.PIN5, pport.PIN6, pport.PIN7)
        self.xyz = [ False] * 3
        self.axy = False
        self.rpy = False
        self.fraction = 1.0

        self.ps3_button_select = False
        self.ps3_button_start = False

#        autorepeat_rate = rospy.get_param( JOY_RATE, 10.0) # Hz
#        self.autorepeat_interval = rospy.Duration.from_sec( 1. / autorepeat_rate).to_sec() # secs

##        self.rosbag_dir = "/root/ros/install/share/arm_robot/data"
##        self.rosbag_done = True
##        self.rosbag_bag = None
##        self.rosbag_sub = None
##        self.rosbag_pub = None

        self.smart_tcp = Float32MultiArray()
        self.smart_tcp.layout.dim.append( MultiArrayDimension( 'translate', 3, 1))
        self.smart_tcp.layout.dim.append( MultiArrayDimension( 'rotate', 3, 1))
        self.smart_tcp.data = [0.0] * 6

        self.ps3 = Ps3()

        moveit_commander.roscpp_initialize( sys.argv)
        rospy.init_node( NODE_NAME, anonymous=False)
        rospy.on_shutdown( self.halt)

        pport.open()

        slaves = soem.open( "enp4s0")
        print( "slaves count = %d\n" % slaves)
        self.slaves = range( 1, slaves + 1)
        for slave in self.slaves:
            soem.reset( slave)
            soem.factor( slave, FACTOR_GRAD)
            soem.enable( slave)

        self.msg_cmd = JointTrajectory()
        self.msg_cmd.joint_names = ['a1_joint', 'a2_joint', 'a3_joint', 'a4_joint', 'a5_joint', 'a6_joint']
        self.msg_cmd.points.append( JointTrajectoryPoint())
        self.msg_cmd.points[0].time_from_start.secs = 0.0
        self.msg_cmd.points[0].time_from_start.nsecs = 100000000.0

##        self.msg_bag = JointTrajectory()
##        self.msg_bag.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
##        self.msg_bag.points.append( JointTrajectoryPoint())
##        self.msg_bag.points[0].time_from_start.secs = 0.0
##        self.msg_bag.points[0].time_from_start.nsecs = 100000000.0

        self.pub_cmd = rospy.Publisher( CONTROLLER_COMMAND, JointTrajectory, queue_size=10)

        self.rosbag = Rosbag( self.pub_cmd)

        self.pub_smart = rospy.Publisher( SMART_TRANSFORMS, Float32MultiArray, queue_size=10)
        self.pub_status = rospy.Publisher( SMART_STATUS, DataList, queue_size=10)

#        self.tf = tf.TransformListener()

#        self.pub_freq = DEFAULT_FREQUENCY
#        self.pub_js = rospy.Publisher( TOPIC_JOINT_STATES, JointState, queue_size=10)

        self.test = True
        self.period = 1.0 / DEFAULT_FREQUENCY
        self.step = math.pi / 60

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander( "manipulator")
#        self.group.set_workspace( [-0.5, -0.5, 0.0, 0.5, 0.5, 1.0])

#        self.msg_js = JointState()
#        self.msg_js.name = self.group.get_joints()

        self.arm_home()

        print( "=> RobotState=%s" % self.robot.get_current_state())
        print( "=> Values=%s" % self.robot.get_current_variable_values())
        print( "=> Group name for 'a1_joint'=%s" % self.robot.get_default_owner_group( 'a1_joint'))
        print( "=> Group names=%s, has 'manipulator'=%s" % ( self.robot.get_group_names(), self.robot.has_group( 'manipulator')))
        print( "=> Joint names=%s" % self.robot.get_joint_names())
        print( "=> Link names=%s" % self.robot.get_link_names())
        print( "=> Root name=%s" % self.robot.get_root_link())
        print( "=> Planning frame=%s" % self.robot.get_planning_frame())

        joint = self.robot.get_joint( 'a1_joint')
        print( "=> Joint name=%s" % joint.name())
        print( "=> Joint value=%s" % joint.value())
        print( "=> Joint variable count=%d" % joint.variable_count())
        print( "=> Joint min bound=%s" % joint.min_bound())
        print( "=> Joint max bound=%s" % joint.max_bound())
        print( "=> Joint bounds=%s" % joint.bounds())

        link = self.robot.get_link( 'a3_link')
        print( "=> Link name=%s" % link.name())
        print( "=> Link pose=%s" % link.pose())

#        self.roll, self.pitch, self.yaw = self.group.get_current_rpy()
#        print( "=> roll=%f, pitch=%f, yaw=%f" % (self.roll, self.pitch, self.yaw))

        self.group.set_named_target( "up")
        self.group.go()

        print( "=> remembed joints=%s" % self.group.get_remembered_joint_values())
        print( "=> variable count=%d" % self.group.get_variable_count())

        '''
        self.group.allow_looking( False)
        self.group.allow_replanning( False)
#        self.group.set_planner_id( "RRT_Connect")
        print( "=> name=<%s>" % self.group.get_name())
        print( "=> ee name=<%s>, has ee=%s" % (self.group.get_end_effector_link(), self.group.has_end_effector_link()))
        print( "=> joint tolerance=%f" % self.group.get_goal_joint_tolerance())
        print( "=> position tolerance=%f" % self.group.get_goal_position_tolerance())
        print( "=> tolerance orientation=%f" % self.group.get_goal_orientation_tolerance())
        print( "=> joints=%s" % self.group.get_joints())
        print( "=> active joints=%s" % self.group.get_active_joints())
        print( "=> constraints=%s" % self.group.get_known_constraints())
        print( "=> path constraints={\n%s}" % self.group.get_path_constraints())
        print( "=> planning frame=<%s>" % self.group.get_planning_frame())
        print( "=> planning time=%f" % self.group.get_planning_time())
        print( "=> end-effector frame=<%s>" % self.group.get_pose_reference_frame())
        print( "=> random joints=%s" % self.group.get_random_joint_values())
        print( "=> random pose=\n%s" % self.group.get_random_pose())
        '''
        '''
        pose_target = self.group.get_current_pose().pose
        pose_target.position.x -= 0.10
        pose_target.position.y -= 0.25
        pose_target.position.z += 0.30
        print( "=> forearm joints=%s" % self.group.set_joint_value_target( pose_target, "ee_link", True))

        self.arm_move_group()
        '''

    def run( self):
        global Shutdown

#        self.arm_home()
        rospy.sleep( 1)

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = -0.5
        p.pose.position.y = -0.0
        p.pose.position.z = 0.5
        print( "box pose=%s" % p)
        self.scene.add_box( "box", p, (0.2, 0.5, 1.0))

        rospy.sleep( 1)

#        rospy.Subscriber( 'joy', Joy, self.callback)
        rospy.Subscriber( JOY_DATA, Joy, self.callback)

        rospy.Subscriber( SMART_JOINT_STATES, JointState, self.smart)

##        while not rospy.is_shutdown():
        while not Shutdown:
            print("\nThe time is %s\n" % rospy.get_rostime())
#            rospy.spin()
            try:
#               try:
#                   ( trans, rot) = listener.lookupTransform( '/base_link', '/wrist_3_link', rospy.Time(0))
#                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                    print( "Ros.run: exception")
#                    continue
                rospy.sleep( 10) # ВАЖНО!!! Нельзя time.sleep( 10)
            except rospy.ROSInterruptException:
                print( "Thread.run is excepted")
                break;

        print( "Thread.run is stoped")


    def motor( self, n, cmd):
#        print( "motor: n=%d, cmd=%s" % (n, cmd))

        if cmd < 0:
            self.motors[n] = True
            pport.pulse( self.pins[n], 500, 20000)
        elif cmd > 0:
            self.motors[n] = True
            pport.pulse( self.pins[n], 2500, 20000)
        else:
            if self.motors[n]:
                self.motors[n] = False
                pport.pulse( self.pins[n], 0, 0)

        return

    def halt( self):
        print( "Thread is halting")
        self.rosbag.rosbag_close()
        moveit_commander.roscpp_shutdown()

        pport.close()

        for slave in self.slaves:
            soem.disable( slave)
        soem.close()

        print( "Thread is halted")

    def callback( self, data):
        if not self.done:
            print( "Action don't complete: skip")
            return

        self.done = False

        if data.buttons[ PS3_BUTTON_SELECT] == 1: # Включение/Выключение записи ROSBAG
            self.ps3_button_select = True # Запомнить факт нажатия SELECT
#            data.buttons[ PS3_BUTTON_SELECT] = 0
        else:
            if self.ps3_button_select: # Обработка события при отжатии SELECT
                self.ps3_button_select = False
##                rc = self.rosbag_make_write()
#                if self.rosbag_sub: # Выключить запись ROSBAG
#                    self.rosbag_stop()
#                else: # Включить запись ROSBAG
#                    self.rosbag_start()
##                self.smart_status_publish( rc)
                self.rosbag.make_write()

        if data.buttons[ PS3_BUTTON_START] == 1: # Включение/Выключение чтения ROSBAG
            self.ps3_button_start = True # Запомнить факт нажатия START
#            data.buttons[ PS3_BUTTON_START] = 0
        else:
            if self.ps3_button_start: # Обработка события при отжатии START
                self.ps3_button_start = False
##                self.smart_status_publish( ROSBAG_READ)
##                rc = self.rosbag_make_read()
##                self.smart_status_publish( rc)
                self.rosbag.make_read()
                del( self.rosbag)
                self.rosbag = Rosbag( self.pub_cmd)
                self.done = True
                return

        ax = data.axes[ PS3_AXIS_STICK_RIGHT_LEFTWARDS]
        ay = data.axes[ PS3_AXIS_STICK_RIGHT_UPWARDS]

        aroll = 0 # Угол крена
        apitch = data.axes[ PS3_AXIS_STICK_LEFT_UPWARDS] # Угол наклона
        ayaw = data.axes[ PS3_AXIS_STICK_LEFT_LEFTWARDS] # Угол рыскания

        if all( button == 0 for button in data.buttons) \
                and (ax == 0 and ay == 0) and (aroll == 0 and apitch == 0 and ayaw == 0):
#            print( "===> ax=%f, ay=%f, work=%s" % (ax, ay, self.work))
            if not self.work:
                self.done = True
                return

        self.work = False

        print( "%d: %s\n%s" %  (data.header.seq, data.axes, data.buttons))
        '''
        x = data.axes[ PS3_AXIS_STICK_LEFT_LEFTWARDS]
        y = data.axes[ PS3_AXIS_STICK_LEFT_UPWARDS]
        if x != 0 or y != 0:
            sx = SPEED_MAX * x
            sy = SPEED_MAX * y
            pose = self.group.get_current_pose().pose
            if self.test:
                pose.position.x -= 0.10
                pose.position.y -= 0.25
#            pose.position.x += sx * self.period
#            pose.position.y += sy * self.period
                pose.position.z += 0.30
            else:
                pose.position.x += 0.10
                pose.position.y += 0.25
                pose.position.z -= 0.30

            print( "=> pose:\n%s" % pose)

#            self.group.set_pose_target( pose)
            self.group.set_position_target( pose)
            plan = self.group.plan()
            self.group.execute( plan, wait=True)

#            self.group.clear_pose_targets()

            self.test = not self.test
        '''

        joints = self.group.get_current_joint_values()
        print( "joints:\n%s" % joints)

        if data.buttons[ PS3_BUTTON_CROSS_RIGHT] == 1:
            self.work = True
#            self.motor( 0, +1)
            joints [0] += self.step
            self.motors[0] = True
            print( "+1: joints[0]=%f\n" % joints[0])
            pport.set_motor( 0, joints[0] * 10)

            soem.goto( 1, radian( joints[0] * 10), radian( math.pi))
        elif data.buttons[ PS3_BUTTON_CROSS_LEFT] == 1:
            self.work = True
#            self.motor( 0, -1)
            joints [0] -= self.step
            self.motors[0] = True
            print( "-1: joints[0]=%f\n" % joints[0])
            pport.set_motor( 0, joints[0] * 10)

            soem.goto( 1, radian( joints[0] * 10), radian( math.pi))
        else:
#            self.motor( 0, 0)
            if self.motors[0]:
                self.motors[0] = False
                print( "=> motor[0] STOP\n")
                pport.stop_motor( 0)

                soem.stop( 1)

        if data.buttons[ PS3_BUTTON_CROSS_DOWN] == 1:
            self.work = True
#            self.motor( 1, +1)
            joints [1] += self.step
            self.motors[1] = True
            print( "+1: joints[1]=%f\n" % joints[1])
            pport.set_motor( 1, joints[1] * 10)
        elif data.buttons[ PS3_BUTTON_CROSS_UP] == 1:
            self.work = True
#            self.motor( 1, -1)
            joints [1] -= self.step
            self.motors[1] = True
            print( "-1: joints[1]=%f\n" % joints[1])
            pport.set_motor( 1, joints[1] * 10)
        else:
#            self.motor( 1, 0)
            if self.motors[1]:
                self.motors[1] = False
                print( "=> motor[1] STOP\n")
                pport.stop_motor( 1)

        if data.buttons[ PS3_BUTTON_REAR_LEFT_2] == 1:
            self.work = True
            joints [2] += self.step
            self.motors[2] = True
            print( "+1: joints[2]=%f\n" % joints[2])
            pport.set_motor( 2, joints[2] * 10)
        elif data.buttons[ PS3_BUTTON_REAR_LEFT_1] == 1:
            self.work = True
            joints [2] -= self.step
            self.motors[2] = True
            print( "-1: joints[2]=%f\n" % joints[2])
            pport.set_motor( 2, joints[2] * 10)
        else:
            if self.motors[2]:
                self.motors[2] = False
                print( "=> motor[2] STOP\n")
                pport.stop_motor( 2)

#!!! Only joints (Remove this fragment later)
#        self.publish_position( joints)
##        self.msg_cmd.header.stamp = rospy.Time.now()
##        self.msg_cmd.points[0].positions = joints
##        self.pub_cmd.publish( self.msg_cmd)

#        self.group.set_joint_value_target( joints)
#        self.group.go()
#        plan = self.group.plan()

#        print( plan)
#        self.group.execute( plan)

#        plan = self.remake_plan( plan)
#        self.publish_plan( plan, self.msg_js)

#        joints = self.group.get_current_pose().pose
#        pose = self.group.get_current_pose().pose
#        print( "=> pose:\n%s" % pose)

##        self.done = True

##        return
#!!!

#        self.group.set_joint_value_target( joints)

        self.doing = False
        pose = self.group.get_current_pose().pose
        wpose = copy.deepcopy( pose)

        if data.buttons[ PS3_BUTTON_ACTION_CIRCLE] == 1:
            self.work = True
            self.doing = True
            self.xyz[ Y] = True
            wpose.position.y -= 0.03
#            self.group.set_position_target( [0.80, -0.20, 0.10])
#            self.group.set_rpy_target( [1.00, 1.00, 1.00])
#            self.group.set_orientation_target( [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        elif data.buttons[ PS3_BUTTON_ACTION_SQUARE] == 1:
            self.work = True
            self.doing = True
            self.xyz[ Y] = True
            wpose.position.y += 0.03
        else:
            if self.xyz[ Y]:
                self.stop = True
                self.xyz[ Y] = False

        if data.buttons[ PS3_BUTTON_ACTION_CROSS] == 1:
            self.work = True
            self.doing = True
            self.xyz[ X] = True
            wpose.position.x -= 0.03
        elif data.buttons[ PS3_BUTTON_ACTION_TRIANGLE] == 1:
            self.work = True
            self.doing = True
            self.xyz[ X] = True
            wpose.position.x += 0.03
        else:
            if self.xyz[ X]:
                self.stop = True
                self.xyz[ X] = False

        if data.buttons[ PS3_BUTTON_REAR_RIGHT_1] == 1:
            self.work = True
            self.doing = True
            self.xyz[ Z] = True
            wpose.position.z += 0.03
        elif data.buttons[ PS3_BUTTON_REAR_RIGHT_2] == 1:
            self.work = True
            self.doing = True
            self.xyz[ Z] = True
            wpose.position.z -= 0.03
        else:
            if self.xyz[ Z]:
                self.stop = True
                self.xyz[ Z] = False

        l = 0.03
        if ax != 0 and ay != 0:
            self.work = True
            self.doing = True
            self.axy = True

            x = float( abs( ax))
            y = float( abs( ay))
            if x < y:
                a = x / y
                y = l / math.sqrt(1 + a*a)
                x = a * y
            else:
                a = y / x
                x = l / math.sqrt(1 + a*a)
                y = a * x

            print( "===> ax=%f, ay=%f, a=%f, x=%f, y=%f, l=%f" % (ax, ay, a, x, y, math.sqrt( x * x + y * y)))

            if ax < 0:
                wpose.position.x += x
            else:
                wpose.position.x -= x

            if ay < 0:
                wpose.position.y -= y
            else:
                wpose.position.y += y
        elif ax == 0 and ay != 0:
            self.work = True
            self.doing = True
            self.axy = True

            if ay < 0:
                wpose.position.y -= l
            else:
                wpose.position.y += l
        elif ax != 0 and ay == 0:
            self.work = True
            self.doing = True
            self.axy = True

            if ax < 0:
                wpose.position.x += l
            else:
                wpose.position.y -= l
        else:
            if self.axy:
                self.stop = True
                self.axy = False

#        if ax != 0 or ay != 0:
#            wpose.orientation.x = 0
#            wpose.orientation.y = 0
#            wpose.orientation.z = 0
#            wpose.orientation.w = 0

        if aroll != 0 or apitch != 0 or ayaw != 0:
            self.work = True
            self.doing = True
            self.rpy = True

#            q = (
#                wpose.orientation.x,
#                wpose.orientation.y,
#                wpose.orientation.z,
#                wpose.orientation.w
#            )
#            roll, pitch, yaw = euler_from_quaternion( q)
            roll, pitch, yaw = self.group.get_current_rpy()
            print( "===> roll=%f, pitch=%f, yaw=%f" % (roll, pitch, yaw))

            if ayaw > 0:
                yaw += self.step
            elif ayaw < 0:
                yaw -= self.step

            if apitch > 0:
                pitch += self.step
            elif apitch < 0:
                pitch -= self.step

            q = quaternion_from_euler( roll, pitch, yaw)
            wpose.orientation.x = q[0]
            wpose.orientation.y = q[1]
            wpose.orientation.z = q[2]
            wpose.orientation.w = q[3]
        else:
            if self.rpy:
                self.stop = True
                self.rpy = False

        if self.doing:
#            wpose.orientation.x = 1
#            wpose.orientation.y = 0
#            wpose.orientation.z = 0
#            wpose.orientation.w = 0

            (plan, fraction) = self.group.compute_cartesian_path(
                               [pose, wpose],   # waypoints to follow
                               0.01,        # eef_step
                               2.0,         # jump_threshold
                               True)
            if fraction < 0.9:
                print( "===> fraction=%f" % fraction)
                self.ps3.rumble( 1, 1.0)
                self.ps3.go( 1)
                self.fraction = fraction
                self.done = True
                return

            if self.fraction < 0.9 and fraction >= 0.9:
                self.ps3.rumble( 1, 0.0)
                self.ps3.go( 0)

            self.fraction = fraction

            print( "=> len( plan)=%d" % len( plan.joint_trajectory.points))
            print( plan.joint_trajectory.points[-1].positions)
            joints = plan.joint_trajectory.points[-1].positions

            pport.set_motor( 0, joints[0] * 10)
            pport.set_motor( 1, joints[1] * 10)
            pport.set_motor( 2, joints[2] * 10)
        elif self.stop:
            self.stop = False
            print( "=> motors STOP\n")
            pport.stop_motor( 0)
            pport.stop_motor( 1)
            pport.stop_motor( 2)
            self.ps3.rumble( 1, 0.0)
            self.ps3.go( 0)

        if self.work:
            self.msg_cmd.header.stamp = rospy.Time.now()
            self.msg_cmd.points[0].positions = joints
            self.pub_cmd.publish( self.msg_cmd)

        self.smart_publish()

        self.done = True

    def smart_publish( self):
        pose = self.group.get_current_pose().pose
        print( "=> smart_publish: pose:\n%s" % pose)
        self.smart_tcp.data[ X] = pose.position.x
        self.smart_tcp.data[ Y] = pose.position.y
        self.smart_tcp.data[ Z] = pose.position.z

        roll, pitch, yaw = self.group.get_current_rpy()
        self.smart_tcp.data[ ROLL] = roll
        self.smart_tcp.data[ PITCH] = pitch
        self.smart_tcp.data[ YAW] = yaw

        self.pub_smart.publish( self.smart_tcp)

    def smart_status_publish( self, rosbag):
        if rosbag == None:
            return

        status = DataList()
        status.tag.append( ROSBAG)
        status.data.append( rosbag)

        self.pub_status.publish( status)

    def smart( self, msg):
        if not self.done:
            print( "!!! smart: Action don't complete: skip")
            return

        self.done = False

        print( "smart: msg")
        print( msg)

        work = False

        joints = self.group.get_current_joint_values()

        for name in msg.name:
            if name == JOINT_NAME_ARRAY [0]:
                if msg.position [0] == STOP_MAGIC:
#                    self.motors [0] = False
                    pport.stop_motor( 0)
                else:
                    work = True
                    joints [0] = msg.position [0]
#                    self.motors [0] = True
                    pport.set_motor( 0, joints[0] * 10)
            elif name == JOINT_NAME_ARRAY [1]:
                if msg.position [1] == STOP_MAGIC:
#                    self.motors [1] = False
                    pport.stop_motor( 1)
                else:
                    work = True
                    joints [1] = msg.position [1]
#                    self.motors [1] = True
                    pport.set_motor( 1, joints[1] * 10)
            elif name == JOINT_NAME_ARRAY [2]:
                if msg.position [2] == STOP_MAGIC:
#                    self.motors [2] = False
                    pport.stop_motor( 2)
                else:
                    work = True
                    joints [2] = msg.position [2]
#                    self.motors [2] = True
                    pport.set_motor( 2, joints[2] * 10)
            elif name == JOINT_NAME_ARRAY [3]:
                if msg.position [3] == STOP_MAGIC:
#                    self.motors [3] = False
                    pport.stop_motor( 3)
                else:
                    work = True
                    joints [3] = msg.position [3]
#                    self.motors [3] = True
                    pport.set_motor( 3, joints[3] * 10)
            elif name == JOINT_NAME_ARRAY [4]:
                if msg.position [4] == STOP_MAGIC:
#                    self.motors [4] = False
                    pport.stop_motor( 4)
                else:
                    work = True
                    joints [4] = msg.position [4]
#                    self.motors [4] = True
                    pport.set_motor( 4, joints[4] * 10)
            elif name == JOINT_NAME_ARRAY [5]:
                if msg.position [5] == STOP_MAGIC:
#                    self.motors [5] = False
                    pport.stop_motor( 5)
                else:
                    work = True
                    joints [5] = msg.position [5]
#                    self.motors [5] = True
                    pport.set_motor( 5, joints[5] * 10)

        print( "smart: joints:\n%s" % joints)

        if work:
            self.msg_cmd.header.stamp = rospy.Time.now()
            self.msg_cmd.points[0].positions = joints
            self.pub_cmd.publish( self.msg_cmd)

        self.done = True

    '''
    def rosbag_write( self, msg):
        if not self.rosbag_done:
            print( "!!! rosbag_write: Action don't complete: skip")
            return

        self.rosbag_done = False

        print( "rosbag_write: msg")
        print( msg)
        self.rosbag_bag.write( JOINT_STATES, msg)

        self.rosbag_done = True

    def rosbag_read( self, topic):
        if not self.rosbag_done:
            print( "!!! rosbag_read: Action don't complete: skip")
            return

        self.rosbag_done = False

        now_start = 0
        n = self.rosbag_bag.get_message_count()
        t_start = self.rosbag_bag.get_start_time()
        t_end = self.rosbag_bag.get_end_time()
        print( "rosbag_read: n=%d, start=%f, end=%f, duration=%f" % (n, t_start, t_end, t_end - t_start))

        for key, msg, ts in self.rosbag_bag.read_messages( topic):
            print( "rosbag_read: topic=%s, start=%f, t.sec=%f, t_nsec=%i" % (key, t_start, ts.to_sec(), ts.to_nsec()))
            print( msg)

            now = rospy.get_rostime()

            if now_start == 0:
                now_start = now

                self.msg_bag.header.stamp = now
                self.msg_bag.points[0].positions = msg.position
                self.pub_cmd.publish( self.msg_bag)
            else:
                t_sleep = ts.to_sec() - t_start - (now - now_start).to_sec()
                if t_sleep > 0:
                    rospy.sleep( t_sleep)

                self.msg_bag.header.stamp = rospy.get_rostime()
                self.msg_bag.points[0].positions = msg.position
                self.pub_cmd.publish( self.msg_bag)

#            rospy.sleep( self.autorepeat_interval / 5.0)

        self.rosbag_done = True

    def rosbag_create( self):
        if not self.rosbag_done:
            print( "!!! rosbag_create: Action don't complete: skip")
            return

        fname = create_fname( self.rosbag_dir, 'bag')
        print( "rosbag_create: fname=%s" % fname)

        self.rosbag_bag = rosbag.Bag( fname, 'w')
        self.rosbag_sub = rospy.Subscriber( JOINT_STATES, JointState, self.rosbag_write)

        data_path = "%s/%s" % ( self.rosbag_dir, "data.bag")
        os.unlink( data_path)
        os.symlink( fname, data_path)

    def rosbag_open( self, fname):
        if self.rosbag_bag:
            return True

        self.rosbag_bag = rosbag.Bag( fname, 'r')
        self.rosbug_pub = rospy.Publisher( CONTROLLER_COMMAND, JointTrajectory, queue_size=10)

    def rosbag_close( self):
        if self.rosbag_sub:
            self.rosbag_sub.unregister()
            self.rosbag_sub = None

        if self.rosbag_bag:
            self.rosbag_bag.close()
            self.rosbag_bag = None

    def rosbag_make_write( self):
        if self.rosbag_sub: # Выключить запись ROSBAG
            self.rosbag_sub.unregister()
            self.rosbag_sub = None

            if self.rosbag_bag:
                self.rosbag_bag.close()
                self.rosbag_bag = None

            return ROSBAG_NONE

        else: # Включить запись ROSBAG
            self.rosbag_create()

            return ROSBAG_WRITE

    def rosbag_make_read( self):
        if self.rosbag_pub: # Выключить чтение ROSBAG
            self.rosbag_pub.unregister()
            self.rosbag_pub = None

            if self.rosbag_bag:
                self.rosbag_bag.close()
                self.rosbag_bag = None

            return ROSBAG_NONE

        else: # Включить чтение ROSBAG
            self.rosbag_open( "%s/%s" % ( self.rosbag_dir, "data.bag"))
            self.rosbag_read( JOINT_STATES)

            return ROSBAG_NONE
    '''

    def remake_plan( self, plan):
        points = plan.joint_trajectory.points
        n = len( points)
        print( "remake_plan: npoints=%d" % n)
        if n == 0:
            return None

        plan.joint_trajectory.points = [points[0], points[-1]]

#        return (points[0], points[-1])
        return (points[n>>1],)

    '''
    def publish_position( self, joints):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAME_ARRAY
        msg.position = joints
#        msg.velocity = self.joints_speed
        msg.velocity = [6.0, 6.0, 6.0, 6.0, 6.0, 6.0]

        self.pub_js.publish( msg)


    def publish_plan( self, points, msg_js):
        if points == None:
            return

        for point in points:
            msg_js.header.stamp = rospy.Time.now()
            msg_js.position = point.positions
            msg_js.velocity = point.velocities

            self.pub_js.publish( msg_js)
    '''

    def arm_home( self):
        print( "=> arm_home")
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.01)

        print( self.group.get_current_joint_values())
        self.group.set_start_state_to_current_state()
        self.group.set_named_target( "up")
        joints = self.group.get_joint_value_target()
        joints[2] = math.pi / 2
#        joints[3] = 0.0
#        joints[4] = math.pi / 2
        print( joints)

#        joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#        self.group.set_joint_value_target( joints)
#        self.group.go()

        self.msg_cmd.header.stamp = rospy.Time.now()
        self.msg_cmd.points[0].positions = joints
        self.pub_cmd.publish( self.msg_cmd)

        self.smart_publish()

        pport.reset_motor( 0)
        pport.reset_motor( 1)
        pport.reset_motor( 2)

        for slave in self.slaves:
            soem.home( slave, HOME_CURRENT_POSITION)
            soem.position( slave)
#            soem.flags( slave, CTRL_RELATE)
            soem.flags( slave, CTRL_IMMEDIATLY)
#            soem.goto( slave, 3600 * 5, 3000)
#            soem.goto( slave, 2 * radian( math.pi) * 5, 2 * radian( math.pi))
#            soem.wait( slave)

    def arm_move_group( self):
        collision_object = moveit_msgs.msg.CollisionObject()

#        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
#        display_trajectory_publisher = rospy.Publisher( '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

#        self.arm_home()

        print( "=> Robot Groups: %s\n" % self.robot.get_group_names())
        print( "=> Printing robot state:\n%s\n" % self.robot.get_current_state())
        print( "=> planning frame: %s" % self.group.get_planning_frame())
        print( "=> end_effector_link: %s" % self.group.get_end_effector_link())
        print( "=> current_pose:\n%s\n" % self.group.get_current_pose().pose)
        print( "=> joint_values: %s" % self.group.get_current_joint_values())
#        print( "=> JointState: %s" % JointState())

#        display_trajectory.trajectory_start = robot.get_current_state()
#        display_trajectory.trajectory.append( plan)
#        display_trajectory_publisher.publish( display_trajectory)

#        group.clear_pose_targets()


class Rosbag( threading.Thread):
    def __init__( self, pub_cmd):
        super( Rosbag, self).__init__()

        self.pub_cmd = pub_cmd
        self.rosbag_dir = "/root/ros/install/share/servo_robot/data"
        self.rosbag_done = True
        self.rosbag_run = False
        self.rosbag_bag = None
        self.rosbag_sub = None

        self.msg_bag = JointTrajectory()
        self.msg_bag.joint_names = ['a1_joint', 'a2_joint', 'a3_joint', 'a4_joint', 'a5_joint', 'a6_joint']
        self.msg_bag.points.append( JointTrajectoryPoint())
        self.msg_bag.points[0].time_from_start.secs = 0.0
        self.msg_bag.points[0].time_from_start.nsecs = 100000000.0

        self.pub_status = rospy.Publisher( SMART_STATUS, DataList, queue_size=10)

    def __del__( self):
        print( "Rosbag.del")
        self.pub_status.unregister()

    def run( self):
        if not self.rosbag_done:
            print( "!!! Rosbag.run: Action don't complete: skip")
            return

        self.rosbag_done = False

        self.rosbag_open( "%s/%s" % ( self.rosbag_dir, "data.bag"))

        self.rosbag_run = True

        self.smart_status_publish( ROSBAG_READ)

        rc = self.rosbag_read( JOINT_STATES) # Главный цикл чтения

        if rc:
            print( "Rosbag.run: Reading is completed")
        else:
            print( "Rosbag.run: Reading is interupted")

        self.smart_status_publish( ROSBAG_NONE)

        self.rosbag_close()

        self.rosbag_done = True

    def rosbag_open( self, fname):
        if self.rosbag_bag:
            return

        self.rosbag_bag = rosbag.Bag( fname, 'r')
        self.rosbug_pub = rospy.Publisher( CONTROLLER_COMMAND, JointTrajectory, queue_size=10)

    def rosbag_close( self):
        if self.rosbag_sub:
            self.rosbag_sub.unregister()
            self.rosbag_sub = None

        if self.rosbag_bag:
            self.rosbag_bag.close()
            self.rosbag_bag = None

    def rosbag_read( self, topic):
        now_start = 0
        n = self.rosbag_bag.get_message_count()
        t_start = self.rosbag_bag.get_start_time()
        t_end = self.rosbag_bag.get_end_time()
        print( "Rosbag.rosbag_read: n=%d, start=%f, end=%f, duration=%f" % (n, t_start, t_end, t_end - t_start))

        for key, msg, ts in self.rosbag_bag.read_messages( topic):
            if not self.rosbag_run:
                return False # Чтение было прервано

            print( "Rosbag.rosbag_read: topic=%s, start=%f, t.sec=%f, t_nsec=%i" % (key, t_start, ts.to_sec(), ts.to_nsec()))
            print( msg)

            now = rospy.get_rostime()

            if now_start == 0:
                now_start = now

                self.msg_bag.header.stamp = now
            else:
                t_sleep = ts.to_sec() - t_start - (now - now_start).to_sec()
                if t_sleep > 0:
                    rospy.sleep( t_sleep)

                self.msg_bag.header.stamp = rospy.get_rostime()

            self.msg_bag.points[0].positions = msg.position
            self.pub_cmd.publish( self.msg_bag)

        self.rosbag_run = False

        return True # Чтение завершилось полностью

    def rosbag_create( self):
        if not self.rosbag_done:
            print( "!!! Rosbag.rosbag_create: Action don't complete: skip")
            return False

        fname = create_fname( self.rosbag_dir, 'bag')
        print( "Rosbag.rosbag_create: fname=%s" % fname)

        self.rosbag_bag = rosbag.Bag( fname, 'w')
        self.rosbag_sub = rospy.Subscriber( JOINT_STATES, JointState, self.rosbag_write)

        data_path = "%s/%s" % ( self.rosbag_dir, "data.bag")
        os.unlink( data_path)
        os.symlink( fname, data_path)

        return True

    def rosbag_write( self, msg):
        if not self.rosbag_done: # Не могу писать при чтении
            print( "!!! Rosbag.rosbag_write: Action don't complete: skip")
            return

        self.rosbag_done = False

        print( "Rosbag.rosbag_write: msg")
        print( msg)
        self.rosbag_bag.write( JOINT_STATES, msg)

        self.rosbag_done = True

    def make_read( self):
        if self.is_alive(): # Выключить чтение ROSBAG
            self.rosbag_run = False
            self.join()
        else: # Включить чтение ROSBAG
            self.start()

    def make_write( self):
        if self.rosbag_sub: # Выключить запись ROSBAG
            self.rosbag_sub.unregister()
            self.rosbag_sub = None

            if self.rosbag_bag:
                self.rosbag_bag.close()
                self.rosbag_bag = None
        else: # Включить запись ROSBAG
            if self.rosbag_create():
                self.smart_status_publish( ROSBAG_WRITE)
                return

        self.smart_status_publish( ROSBAG_NONE)

    def smart_status_publish( self, rosbag):
        if rosbag == None:
            return

        status = DataList()
        status.tag.append( ROSBAG)
        status.data.append( rosbag)

        self.pub_status.publish( status)


class Ps3:
    def __init__( self):
        self.stat = None

        self.ps3 = JoyFeedbackArray()

        self.led1 = JoyFeedback()
        self.led1.type = TYPE_LED
        self.led1.id = PS3_LED_1
        self.led1.intensity = 0.0

        self.led2 = JoyFeedback()
        self.led2.type = TYPE_LED
        self.led2.id = PS3_LED_2
        self.led2.intensity = 0.0

        self.led3 = JoyFeedback()
        self.led3.type = TYPE_LED
        self.led3.id = PS3_LED_3
        self.led3.intensity = 0.0

        self.led4 = JoyFeedback()
        self.led4.type = TYPE_LED
        self.led4.id = PS3_LED_4
        self.led4.intensity = 0.0

        self.ps3.array.append( self.led1)
        self.ps3.array.append( self.led2)
        self.ps3.array.append( self.led3)
        self.ps3.array.append( self.led4)

        self.rumble1 = JoyFeedback()
        self.rumble1.type = TYPE_RUMBLE
        self.rumble1.id = PS3_RUMBLE_1
        self.rumble1.intensity = 0.0

        self.ps3.array.append( self.rumble1)

        print( "=== ps3 ===")
        print( self.ps3)

        self.pub = rospy.Publisher( 'joy/set_feedback', JoyFeedbackArray, queue_size=2)

    def led( self, n, value):
        if value != 0:
            value = 1.0

        if n == 1:
            self.led1.intensity = value
        elif n == 2:
            self.led2.intensity = value
        elif n == 3:
            self.led3.intensity = value
        elif n == 4:
            self.led4.intensity = value
        else:
            print( "led: wrong led number <%d>" % n)

    def rumble( self, n, value):
        if value != 0:
            value = 1.0

        if n == 1:
            self.rumble1.intensity = value
        else:
            print( "rumble: wrong rumble number <%d>" % n)

    def publish( self):
        self.pub.publish( self.ps3)

    def go( self, stat):
        if self.stat == stat:
            return

        print( "go: self.stat=%s, stat=%s" % ( self.stat, stat))
        self.pub.publish( self.ps3)
#        rospy.sleep( 0.1)
#        self.pub.publish( self.ps3)
        self.stat = stat


class Talker( threading.Thread):
    def __init__( self, interval):
        super( Talker, self).__init__()
        self.daemon = True
        self.ready = True
        self.interval = interval # Hz
        self.ps3 = Ps3()

    def run( self):
        r = rospy.Rate( self.interval)

        while not rospy.is_shutdown():
            try:
                if self.ready:
                    self.ps3.led( 3, 1.0)
                    self.ps3.led( 4, 1.0)
                    self.ps3.rumble( 1, 1.0)

                    self.ps3.publish()
                else:
                    self.ps3.led( 3, 0.0)
                    self.ps3.led( 4, 0.0)
                    self.ps3.rumble( 1, 0.0)

                    self.ps3.publish()

                self.ready = not self.ready

                r.sleep()
            except rospy.ROSInterruptException:
                print( "Thread.run is excepted")
                break;

        print( "Talker.run is stoped")



def shutdown( nsig, frame):
    global Shutdown

    print( "shutdown")
    Shutdown = True

if __name__ == '__main__':
    global Shutdown

    Shutdown = False
    signal.signal( signal.SIGTERM, shutdown)
    signal.signal( signal.SIGINT, shutdown)

    listener = Listener()
    listener.start()

#    talker = Talker( 2)
#    talker.start()

    while not rospy.is_shutdown():
        try:
            rospy.sleep( 10) # ВАЖНО!!! Нельзя time.sleep( 10)
        except rospy.ROSInterruptException:
            print( "Main is excepted")
            break;

    listener.join()
#    talker.join()

    print( "Main is exited")
