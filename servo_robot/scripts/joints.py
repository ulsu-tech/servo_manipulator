#!/usr/bin/env python

import rospy

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose

def scale_speed( traj, scale):
    # Create a new trajectory object
    new_traj = RobotTrajectory()

    # Initialize the new trajectory to be the same as the input trajectory
    new_traj.joint_trajectory = traj.joint_trajectory

    # Get the number of joints involved
    n_joints = len( traj.joint_trajectory.joint_names)

    # Get the number of points on the trajectory
    n_points = len( traj.joint_trajectory.points)

    # Store the trajectory points
    points = list( traj.joint_trajectory.points)

    # Cycle through all points and joints and scale the time from start,
    # speed and acceleration
    for i in range( n_points):
        point = JointTrajectoryPoint()

        # The joint positions are not scaled so pull them out first
        point.positions = traj.joint_trajectory.points[i].positions

        # Next, scale the time_from_start for this point
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale

        # Get the velocities for each joint for this point
        point.velocities = list( traj.joint_trajectory.points[i].velocities)

        # Get the accelerations for each joint for this point
        point.accelerations = list( traj.joint_trajectory.points[i].accelerations)

        # Scale the velocity and acceleration for each joint at this point
        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale

        # Store the scaled trajectory point
        points[i] = point

    new_traj.joint_trajectory.points = points

    # Return the new trajecotry
    return new_traj
