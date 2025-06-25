#!/usr/bin/env python3

import rospy
from hmi.msg import ControlCommand
import std_msgs

class HMICommandListener:
    #def __init__(self):
    #    rospy.init_node('hmi_command_listener', anonymous=True)
    #    rospy.Subscriber('hmi/user_command', ControlCommand, self.command_callback)
    #    rospy.loginfo("[HMI Listener] Node started. Waiting for HMI commands...")

    #    self.system_command_pub = rospy.Publisher('hmi/system_command', ControlCommand, queue_size=10)

    #    self.light_cons_on = rospy.Publisher('light_indication/constant/on', std_msgs.msg.String, queue_size=10)
    #    self.light_blink_on = rospy.Publisher('light_indication/blink/on', std_msgs.msg.String, queue_size=10)
    #    self.light_off = rospy.Publisher('light_indication/off', std_msgs.msg.String, queue_size=10)
    #    self.constant = False  # Default to single mode

    def __init__(self, rospy, ros_handler):
        self.rospy = rospy
        self.rosHandler = ros_handler

    def light_indicator_cons(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.rosHandler.publish('light_indication/constant/on', msg)

    def light_indicator_blink(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.rosHandler.publish('light_indication/blink/on', msg)

    def light_indicator_off(self, color):
        msg = std_msgs.msg.String()
        msg.data = color
        self.rosHandler.publish('light_indication/off', msg)

    def in_progress(self):
        rospy.loginfo("[Action] Starting sorting process...")
        # TODO: Add actual logic here
        self.light_indicator_cons("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        #rospy.sleep(0.1)
        #self.light_indicator_cons("BUZZER")
        #rospy.sleep(1)
        #self.light_indicator_off("BUZZER")

    def stop_system(self):
        rospy.loginfo("[Action] Stopping system...")
        # TODO: Add actual logic here
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_blink("ORANGE")

    def reset_indication(self):
        rospy.loginfo("[Action] Resetting indication...")
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
        #rospy.sleep(0.1)
        #self.light_indicator_blink("BUZZER")

    def error(self):
        rospy.logerr("[Action] ERROR state activated!")

        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_blink("RED")

