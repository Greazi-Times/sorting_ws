#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String

class Transportsystem:
    def __init__(self, rospy, ros_handler):
        # Publishers en subscribers
        #self.transport_pub = rospy.Publisher('/transport_command', Bool, queue_size=10)
        #self.begin_sub = rospy.Subscriber('/object_op_begin_transport', Bool, self._beginsensor_callback)
        #self.eind_sub = rospy.Subscriber('/object_op_eind_transport', Bool, self._eindsensor_callback)

        self.rospy = rospy
        self.ros_handler = ros_handler

        # Interne status
        self.transport_actief = False
        self.object_klaar = False
        self.object_aangekomen = False
        self.eindsensor_bezet = False

    def _beginsensor_callback(self, msg):
        self.object_klaar = msg.data

    def _eindsensor_callback(self, msg):
        self.eindsensor_bezet = msg.data
        if self.transport_actief and msg.data:
            self.object_aangekomen = True

    def start_transport(self):
        self.transport_actief = True
        self.ros_handler.publish('transportsystem/command', Bool(data=True))
        self.rospy.loginfo("Transportband gestart.")

    def stop_transport(self, via_noodstop=False):
        if self.transport_actief:
            self.transport_actief = False
            self.ros_handler.publish('transportsystem/command', Bool(data=False))
            self.rospy.loginfo("Transportband gestopt.")

            # Status feedback
            status_msg = String()
            status_msg.data = "gestopt via noodstop" if via_noodstop else "gestopt normaal"

    def perform(self):
        rospy.loginfo("Perform gestart: wachten op object aan beginsensor...")
        self.object_klaar = False
        self.object_aangekomen = False
        rate = self.rospy.Rate(2)

        while not self.rospy.is_shutdown():
            if self.object_klaar and not self.eindsensor_bezet:
                self.rospy.loginfo("Beginsensor actief en eindsensor vrij. Start transport.")
                break
            elif self.eindsensor_bezet:
                self.rospy.logwarn("Eindsensor bezet, wacht tot deze vrij is...")
            rate.sleep()

        self.start_transport()
        self.rospy.loginfo("Transportband draait. Wachten op eindsensor...")

        while not self.rospy.is_shutdown():
            if self.object_aangekomen:
                self.rospy.loginfo("Eindsensor geactiveerd. Stop transport.")
                break
            rate.sleep()

        self.stop_transport()
        self.rospy.loginfo("Transport voltooid.")
        return True

    def foutiefObject(self):
        self.rospy.loginfo("Afvoer foutobject gestart. Band draait tot eindsensor vrij is.")
        self.start_transport()
        rate = self.rospy.Rate(2)

        while not self.rospy.is_shutdown():
            if not self.eindsensor_bezet:
                self.rospy.loginfo("Eindsensor vrij. Foutobject afgevoerd.")
                break
            rate.sleep()

        self.stop_transport()
        return True

    def noodstop(self):
        self.rospy.logwarn("NOODSTOP lokaal geactiveerd!")
        self.stop_transport(via_noodstop=True)
        return True
