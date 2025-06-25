#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from my_demo.msg import MoveToPoseAction, MoveToPoseGoal

def send_goal():
    rospy.init_node('goal_sender')

    # Maak client voor MoveToPose action
    client = actionlib.SimpleActionClient('move_to_pose', MoveToPoseAction)
    rospy.loginfo("Wachten op action server...")
    client.wait_for_server()
    rospy.loginfo("Verbonden met action server")

    # Stel doelpositie in (bereikbaar voor Lite6)
    goal = MoveToPoseGoal()
    goal.target_pose.position.x = 0.076
    goal.target_pose.position.y = 0.306
    goal.target_pose.position.z = 0.365

    # Oriëntatie: 180° rotatie om X-as (Z-as wijst omlaag)
    roll = math.radians(180)
    pitch = 0
    yaw = math.radians(-90)
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    goal.target_pose.orientation.x = qx
    goal.target_pose.orientation.y = qy
    goal.target_pose.orientation.z = qz
    goal.target_pose.orientation.w = qw

    rospy.loginfo("Doel gestuurd: x=%.2f y=%.2f z=%.2f", 
                  goal.target_pose.position.x,
                  goal.target_pose.position.y,
                  goal.target_pose.position.z)

    # Verstuur doel en wacht op resultaat
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Beweging voltooid. Succes: %s", result.success)

if __name__ == '__main__':
    send_goal()

