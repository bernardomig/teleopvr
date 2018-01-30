#!/usr/bin/env python2

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("manipulator")


display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20)

rospy.sleep(1)

group.set_planning_time(10)

group.set_joint_value_target([0, 0, 0, 0, -pi / 2, 0])

group.go()

print group.get_current_pose()

pose = geometry_msgs.msg.Pose()

pose.orientation.x = 0.0
pose.orientation.y = 1
pose.orientation.z = 0.0
pose.orientation.w = 0
pose.position.x = 0.35
pose.position.y = 0
pose.position.z = 0

waypoints = []

for z in np.linspace(0.002, 0.3, 3):
    pose.position.z = z
    waypoints.append(copy.deepcopy(pose))
    pose.position.y = -0.2
    waypoints.append(copy.deepcopy(pose))
    pose.position.x = 0.6
    waypoints.append(copy.deepcopy(pose))
    pose.position.y = 0.2
    waypoints.append(copy.deepcopy(pose))
    pose.position.x = 0.4
    waypoints.append(copy.deepcopy(pose))
    pose.position.y = 0
    waypoints.append(copy.deepcopy(pose))


(plan3, fraction) = group.compute_cartesian_path(
    waypoints,
    0.01,
    0.0)

if fraction == 1.0:
    print 'planning was successful'
    raw_input()
    group.execute(plan3)
else:
    print 'planning was not successfull'
    print fraction
