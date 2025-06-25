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

    def _beginsensor_callback(self, msg):
        self.object_klaar = msg.data

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

            # Status feedback
            status_msg = String()
            status_msg.data = "gestopt via noodstop" if via_noodstop else "gestopt normaal"

    def perform(self):
        rate = rospy.Rate(2)

        if self.eindsensor_bezet:
            rospy.logerr("Eindsensor is al bezet bij start. Kan niet beginnen.")
            return False
        
        # Check voor x aantal seconden of beginsensor actief is
        start_time = rospy.get_time()
        timeout = 5  # seconds

        while not rospy.is_shutdown():
            if self.object_klaar:
                break
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("Geen object gedetecteerd bij beginsensor na 5 seconden.")
                return False
            rate.sleep()

        self.rospy.loginfo("Object gedetecteerd transportband start.")
        self.start_transport()

        while not self.rospy.is_shutdown():
            if self.eindsensor_bezet:
                break
            rate.sleep()

        rospy.loginfo("Object aangekomen bij eindsensor. Transport stopt.")
        self.stop_transport()
        return True

    def foutiefObject(self):
        rospy.loginfo("Afvoer foutobject gestart. Band draait tot eindsensor vrij is.")
        self.start_transport()
        rate = self.rospy.Rate(2)

        while not self.rospy.is_shutdown():
            if not self.eindsensor_bezet:
                rospy.loginfo("Eindsensor vrij. Foutobject afgevoerd.")
                break
            rate.sleep()

        self.stop_transport()
        return True

    def noodstop(self):
        self.stop_transport(via_noodstop=True)
        return True
