#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hmi.msg import ControlCommand
import std_msgs

class HMICommandListener:
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
        self.light_indicator_cons("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        # Optional:
        # self.light_indicator_cons("BUZZER")
        # rospy.sleep(1)
        # self.light_indicator_off("BUZZER")

    def stop_system(self):
        rospy.loginfo("[Action] Stopping system...")
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_blink("ORANGE")

    def reset_indication(self):
        rospy.loginfo("[Action] Resetting indication...")
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("RED")
        rospy.sleep(0.1)
        self.light_indicator_cons("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_off("BUZZER")

    def emergency_stop(self):
        rospy.logerr("[Action] EMERGENCY STOP activated!")
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_cons("RED")
        # Optional:
        # self.light_indicator_blink("BUZZER")

    def error(self):
        rospy.logerr("[Action] ERROR state activated!")
        self.light_indicator_off("ORANGE")
        rospy.sleep(0.1)
        self.light_indicator_off("GREEN")
        rospy.sleep(0.1)
        self.light_indicator_blink("RED")
