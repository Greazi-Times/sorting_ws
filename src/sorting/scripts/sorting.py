#!/usr/bin/env python3

import rospy
from hmi.msg import ControlCommand
import std_msgs

class HMICommandListener:
    def __init__(self):
        rospy.init_node('hmi_command_listener', anonymous=True)
        rospy.Subscriber('/hmi/user_command', ControlCommand, self.command_callback)
        rospy.loginfo("[HMI Listener] Node started. Waiting for HMI commands...")
        self.command_pub = rospy.Publisher('/light_indication/command', ControlCommand, queue_size=10)

        self.light_cons_on = rospy.Publisher('light_indication/constant/on', std_msgs.msg.String, queue_size=10)
        self.light_blink_on = rospy.Publisher('light_indication/blink/on', std_msgs.msg.String, queue_size=10)
        self.light_off = rospy.Publisher('light_indication/off', std_msgs.msg.String, queue_size=10)
        self.constant = False  # Default to single mode

    def command_callback(self, msg):
        rospy.loginfo(f"[HMI Listener] Received command: {msg.command}")

        # Example actions
        if msg.command == "START":
            self.start_sorting()
        elif msg.command == "STOP":
            self.stop_system()
        elif msg.command == "RESET":
            self.reset_system()
        elif msg.command == "EMERGENCY_STOP":
            self.emergency_stop()
        elif msg.command == "CONSTANT":
            self.constant = True
        elif msg.command == "SINGLE":
            self.constant = False
        else:
            rospy.logwarn(f"[HMI Listener] Unknown command: {msg.command}")

    def light_indicator_cons(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.light_cons_on.publish(msg)

    def light_indicator_blink(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.light_blink_on.publish(msg)

    def light_indicator_off(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.light_off.publish(msg)

    def start_sorting(self):
        rospy.loginfo("[Action] Starting sorting process...")
        # TODO: Add actual logic here
        self.light_indicator_cons("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_cons("BUZZER")
        rospy.sleep(1)
        self.light_indicator_off("BUZZER")

    def stop_system(self):
        rospy.loginfo("[Action] Stopping system...")
        # TODO: Add actual logic here
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_blink("ORANGE")

    def reset_system(self):
        rospy.loginfo("[Action] Resetting system...")
        # TODO: Add actual logic here
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("RED")
        rospy.sleep(0.1)
        self.light_indicator_cons("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_off("BUZZER")

    def emergency_stop(self):
        rospy.logerr("[Action] EMERGENCY STOP activated!")
        # TODO: Add actual logic here
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_cons("RED")
        rospy.sleep(0.1)
        self.light_indicator_blink("BUZZER")

if __name__ == '__main__':
    try:
        HMICommandListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
