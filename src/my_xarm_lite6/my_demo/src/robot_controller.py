#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import actionlib

from geometry_msgs.msg import Pose
from my_demo.msg import MoveToPoseAction, MoveToPoseFeedback, MoveToPoseResult
from moveit_msgs.msg import MoveGroupAction
from xarm_msgs.msg import RobotMsg

class MoveToPoseActionServer:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('robot_controller')

        rospy.loginfo("Wachten tot move_group actief is...")
        move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        move_group_client.wait_for_server()
        rospy.loginfo("move_group server actief.")

        self.arm_group = moveit_commander.MoveGroupCommander("arm")

        self.server = actionlib.SimpleActionServer(
            'move_to_pose',
            MoveToPoseAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("MoveToPose action server gestart.")

        self.estop_active = False
        self._estop_logged = False

        rospy.Subscriber("/ufactory/robot_states", RobotMsg, self.robot_state_callback)
        rospy.Timer(rospy.Duration(0.1), self.check_estop_timer)

    def robot_state_callback(self, msg):
        self.estop_active = (msg.err == 2)

    def check_estop_timer(self, event):
        if self.estop_active and not self._estop_logged:
            rospy.logwarn("E-stop actief: robotbeweging wordt direct gestopt!")
            self.arm_group.stop()
            self._estop_logged = True
        elif not self.estop_active:
            self._estop_logged = False

    def execute_callback(self, goal):
        feedback = MoveToPoseFeedback()
        result = MoveToPoseResult()

        # --- Status: planning ---
        feedback.status = "planning"
        self.server.publish_feedback(feedback)
        rospy.loginfo("Status: planning")

        # Check E-stop vóór beweging
        if self.estop_active:
            rospy.logwarn("Beweging afgebroken: E-stop actief vóór uitvoering")
            feedback.status = "cancelled"
            self.server.publish_feedback(feedback)

            result.success = False
            result.status = "cancelled"
            self.server.set_aborted(result)
            return

        # --- Set doelpositie ---
        self.arm_group.set_pose_target(goal.target_pose)

        # --- Status: moving ---
        feedback.status = "moving"
        self.server.publish_feedback(feedback)
        rospy.loginfo("Status: moving")

        success = self.arm_group.go(wait=True)

        # --- Stop + clear targets ---
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            feedback.status = "op_positie"
            self.server.publish_feedback(feedback)
            rospy.loginfo("Status: op_positie")
            rospy.sleep(0.2)

            result.success = True
            result.status = "op_positie"
            self.server.set_succeeded(result)
        else:
            feedback.status = "error"
            self.server.publish_feedback(feedback)
            rospy.logwarn("Status: error")

            result.success = False
            result.status = "error"
            self.server.set_aborted(result)

if __name__ == '__main__':
    try:
        server = MoveToPoseActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

