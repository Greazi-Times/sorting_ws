#!/usr/bin/env python3

import rospy
from hmi.msg import ControlCommand
from std_msgs.msg import String  # Add this at the top

prefix = "[HMI ROS Handler] "

class ROSHandler:
    def __init__(self):
        self.command_pub = rospy.Publisher('hmi/user_command', ControlCommand, queue_size=10)
        rospy.loginfo("[ROSHandler] ROS handler initialized and publisher ready.")

        self.listener_callback = None  # Will hold user-defined callback
        self.command_sub = rospy.Subscriber('hmi/system_command', ControlCommand, self._internal_callback)

    def publish_control_command(self, command_str):
        msg = ControlCommand()
        msg.command = command_str
        rospy.loginfo(f"[ROSHandler] Publishing command: {msg.command}")
        self.command_pub.publish(msg)

    def _internal_callback(self, msg):
        rospy.loginfo(f"[ROSHandler] Received command: {msg.command}")
        if self.listener_callback:
            self.listener_callback(msg)

    def add_listener(self, callback):
        rospy.loginfo("[ROSHandler] Listener callback registered.")
        self.listener_callback = callback
