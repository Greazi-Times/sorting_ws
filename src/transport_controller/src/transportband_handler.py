#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool, String
import threading

class Transportband:
    def __init__(self):
        # Publishers en subscribers
        self.transport_pub = rospy.Publisher('/transport_command', Bool, queue_size=10)
        self.status_pub = rospy.Publisher('/transport_status', String, queue_size=10)
        self.begin_sub = rospy.Subscriber('/object_op_begin_transport', Bool, self._beginsensor_callback)
        self.eind_sub = rospy.Subscriber('/object_op_eind_transport', Bool, self._eindsensor_callback)
        self.noodstop_sub = rospy.Subscriber('/transport/estop', Bool, self._noodstop_callback)

        # Interne status
        self.transport_actief = False
        self.object_klaar = False
        self.object_aangekomen = False
        self.eindsensor_bezet = False
        self.noodstop_event = threading.Event()

    def _beginsensor_callback(self, msg):
        self.object_klaar = msg.data
        if msg.data and not self.transport_actief:
            rospy.loginfo("Beginsensor: object gedetecteerd.")

    def _eindsensor_callback(self, msg):
        self.eindsensor_bezet = msg.data
        if self.transport_actief and msg.data:
            self.object_aangekomen = True
            rospy.loginfo("Eindsensor: object aangekomen.")

    def _noodstop_callback(self, msg):
        if msg.data:
            rospy.logwarn("NOODSTOP ontvangen via topic!")
            self.noodstop_event.set()
            self.stop_transport(via_noodstop=True)

    def start_transport(self):
        self.transport_actief = True
        self.transport_pub.publish(Bool(data=True))
        rospy.loginfo("Transportband gestart.")

    def stop_transport(self, via_noodstop=False):
        if self.transport_actief:
            self.transport_actief = False
            self.transport_pub.publish(Bool(data=False))
            rospy.loginfo("Transportband gestopt.")

            # Status feedback
            status_msg = String()
            status_msg.data = "gestopt via noodstop" if via_noodstop else "gestopt normaal"
            self.status_pub.publish(status_msg)

    def perform(self):
        rospy.loginfo("Perform gestart: wachten op object aan beginsensor...")
        self.noodstop_event.clear()
        self.object_klaar = False
        self.object_aangekomen = False
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.noodstop_event.is_set():
                rospy.logwarn("Transport onderbroken door noodstop tijdens wachtfase.")
                return False
            if self.object_klaar and not self.eindsensor_bezet:
                rospy.loginfo("Beginsensor actief en eindsensor vrij. Start transport.")
                break
            elif self.eindsensor_bezet:
                rospy.logwarn("Eindsensor bezet, wacht tot deze vrij is...")
            rate.sleep()

        self.start_transport()
        rospy.loginfo("Transportband draait. Wachten op eindsensor...")

        while not rospy.is_shutdown():
            if self.noodstop_event.is_set():
                rospy.logwarn("Transport onderbroken door noodstop tijdens transport.")
                return False
            if self.object_aangekomen:
                rospy.loginfo("Eindsensor geactiveerd. Stop transport.")
                break
            rate.sleep()

        self.stop_transport()
        rospy.loginfo("Transport voltooid.")
        return True

    def foutiefObject(self):
        rospy.loginfo("Afvoer foutobject gestart. Band draait tot eindsensor vrij is.")
        self.noodstop_event.clear()
        self.start_transport()
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.noodstop_event.is_set():
                rospy.logwarn("Foutobject-afvoer gestopt door noodstop.")
                return False
            if not self.eindsensor_bezet:
                rospy.loginfo("Eindsensor vrij. Foutobject afgevoerd.")
                break
            rate.sleep()

        self.stop_transport()
        return True

    def noodstop(self):
        rospy.logwarn("NOODSTOP lokaal geactiveerd!")
        self.noodstop_event.set()
        self.stop_transport(via_noodstop=True)
        return True
