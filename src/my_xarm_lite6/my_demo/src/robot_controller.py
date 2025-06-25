#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib

from geometry_msgs.msg import Pose
from my_demo.msg import MoveToPoseAction, MoveToPoseFeedback, MoveToPoseResult
from moveit_msgs.msg import MoveGroupAction

class MoveToPoseActionServer:
    def __init__(self):
        # Start MoveIt en ROS node
        moveit_commander.roscpp_initialize([])
        rospy.init_node('robot_controller')

        # Wacht tot de move_group server actief is (belangrijk voor planning)
        rospy.loginfo("Wachten tot move_group actief is...")
        move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        move_group_client.wait_for_server()
        rospy.loginfo("move_group server actief.")

        # Aanmaken van eenMoveGroupCommander voor de 'arm' groep
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        rospy.loginfo("Verbonden met planning group 'arm'")

        # Start een Action Server voor doelen die via MoveToPoseAction gestuurd worden
        self.server = actionlib.SimpleActionServer(
            'move_to_pose',               # Naam van de action server
            MoveToPoseAction,             # Het actietype
            execute_cb=self.execute_cb,   # Callback functie die uitgevoerd wordt bij een nieuw doel
            auto_start=False              # Start pas handmatig
        )
        self.server.start()
        rospy.loginfo("MoveToPoseAction server gestart")

    def execute_cb(self, goal):
        # Ontvangen van een nieuw doel (positie + oriëntatie)
        rospy.loginfo("Nieuw doel ontvangen")

        # Zet dit doel als het gewenste eindpunt voor de robotarm
        self.arm_group.set_pose_target(goal.target_pose)

        # Start planning en uitvoering van de beweging
        success = self.arm_group.go(wait=True)

        # Stop eventuele restbeweging en wis het doel
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        # Stuur 100% voortgang terug als feedback
        feedback = MoveToPoseFeedback()
        feedback.progress = 100.0
        self.server.publish_feedback(feedback)

        # Stuur resultaat terug naar de client
        result = MoveToPoseResult()
        result.success = success
        if success:
            rospy.loginfo("Beweging succesvol uitgevoerd.")
            self.server.set_succeeded(result)
        else:
            rospy.logwarn("Beweging mislukt.")
            self.server.set_aborted(result)

if __name__ == '__main__':
    try:
        # Start de action server en blijf actief
        server = MoveToPoseActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

