#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
from std_msgs.msg import Bool, String

class Transportsystem:
    def __init__(self, rospy, ros_handler):
        self.rospy = rospy
        self.ros_handler = ros_handler

        self.transport_actief = False
        self.beginsensor_bezet = False
        self.eindsensor_bezet = False

        self.motor_status = ""
        self.noodstop_event = threading.Event()  # << Thread-safe noodstop

        rospy.Subscriber("transportband/motor", String, self.motor_status_callback)

    def motor_status_callback(self, msg):
        self.motor_status = msg.data
        if msg.data == "motor gestopt":
            self.rospy.logwarn("Transportband gestopt.")
        elif msg.data == "motor gestart":
            self.rospy.loginfo("Transportband gestart.")

    def _beginsensor_callback(self, msg):
        self.beginsensor_bezet = msg.data

    def _eindsensor_callback(self, msg):
        self.eindsensor_bezet = msg.data

    def start_transport(self):
        self.transport_actief = True
        self.ros_handler.publish('transportsystem/command', Bool(data=True))
        self.rospy.loginfo("Transportband gestart.")

    def stop_transport(self, via_noodstop=False):
        if self.transport_actief:
            self.transport_actief = False
            self.ros_handler.publish('transportsystem/command', Bool(data=False))
            self.rospy.loginfo("Transportband gestopt.")

            status_msg = String()
            status_msg.data = "gestopt via noodstop" if via_noodstop else "gestopt normaal"
            # self.ros_handler.publish('transportsystem/status', status_msg)

    def perform(self):
        rate = rospy.Rate(2)

        if self.eindsensor_bezet:
            rospy.logerr("Eindsensor is al bezet bij start. Kan niet beginnen.")
            return False

        if self.noodstop_event.is_set():
            rospy.logwarn("Noodstop is actief — transport wordt niet gestart.")
            return False

        start_time = rospy.get_time()
        timeout_start = 5

        while not rospy.is_shutdown():
            if self.noodstop_event.is_set():
                while self.motor_status != "motor gestopt" and not rospy.is_shutdown():
                    self.stop_transport(via_noodstop=True)
                    rate.sleep()
                rospy.logwarn("Noodstop ingedrukt, transport gestopt.")
                return False
            if self.beginsensor_bezet:
                break
            if rospy.get_time() - start_time > timeout_start:
                rospy.logwarn("Geen object gedetecteerd bij beginsensor na 5 seconden.")
                return False
            rate.sleep()

        self.start_transport()

        stop_time = rospy.get_time()
        timeout_stop = 8

        while not rospy.is_shutdown():
            if self.noodstop_event.is_set():
                while self.motor_status != "motor gestopt" and not rospy.is_shutdown():
                    self.stop_transport(via_noodstop=True)
                    rate.sleep()
                    return False
                rospy.logwarn("Noodstop ingedrukt, transport gestopt.")
                return False
            if self.eindsensor_bezet:
                break
            if rospy.get_time() - stop_time > timeout_stop:
                rospy.logwarn("Geen object gedetecteerd bij eindsensor na 8 seconden.")
                return False
            rate.sleep()

        rospy.loginfo("Object aangekomen bij eindsensor. Transport stopt.")
        self.stop_transport()
        return True

    def foutiefObject(self):
        rospy.loginfo("Afvoer foutobject gestart. Band draait tot eindsensor vrij is.")
        self.start_transport()
        rate = self.rospy.Rate(2)

        while not self.rospy.is_shutdown():
            if self.noodstop_event.is_set():
                while self.motor_status != "motor gestopt" and not rospy.is_shutdown():
                    self.stop_transport(via_noodstop=True)
                    rate.sleep()
                    return False
                self.rospy.logwarn("Noodstop ingedrukt, transport gestopt.")
                return False
            if not self.eindsensor_bezet:
                rospy.loginfo("Eindsensor vrij. Foutobject afgevoerd.")
                break
            rate.sleep()

        if not self.noodstop_event.is_set():
            self.rospy.sleep(1)
            self.stop_transport()
        return True

    def noodstop(self):
        self.noodstop_event.set()
        if self.motor_status == "motor gestopt":
            return True

    def reset(self):
        if self.motor_status == "motor gestopt":
            self.noodstop_event.clear()
            self.rospy.loginfo("Noodstop gereset in transportsysteem.")
            return True
        else:
            self.rospy.logwarn("Reset geblokkeerd: motor draait nog op transport.")