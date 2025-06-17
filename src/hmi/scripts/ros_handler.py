#!/usr/bin/env python3

import rospy
from hmi.msg import ControlCommand
from std_msgs.msg import String  # Add this at the top

class ROSHandler:
    def __init__(self):
        self.command_pub = rospy.Publisher('hmi/user_command', ControlCommand, queue_size=10)
        rospy.loginfo("[ROSHandler] ROS handler initialized and publisher ready.")

    def publish_control_command(self, command_str):
        msg = ControlCommand()
        msg.command = command_str
        rospy.loginfo(f"[ROSHandler] Publishing command: {msg.command}")
        self.command_pub.publish(msg)

    def system_status_callback(self, msg):
        status = msg.data
        print(f"[ROS] System status update: {status}")

        if status == "ERROR":
            self.red_indicator.blink()
            self.green_indicator.off()
            self.orange_indicator.off()
        elif status == "IDLE":
            self.green_indicator.on()
            self.red_indicator.off()
            self.orange_indicator.off()
        elif status == "RUNNING":
            self.orange_indicator.on()
            self.green_indicator.off()
            self.red_indicator.off()
        else:
           print("[WARN] Unknown system status received:", status)


    def setup_ros_subscribers(self):
        rospy.Subscriber("hmi/system_status", std, self.system_status_callback)
