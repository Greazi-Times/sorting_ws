#!/usr/bin/env python

"""
    mondhygiene_client.py
    Purpose: Roept de Mondhygiene_detectie service aan en ontvangt A, B, angle en object_class.
    Author: Tessa van Lankveld
    Versie: 1.0
"""

import rospy
from depthai_ros_msgs.srv import camera, cameraRequest

class MondhygieneClient:
    def __init__(self):
        rospy.wait_for_service('/get_detection_info')
        try:
            self.client = rospy.ServiceProxy('/get_detection_info', camera)
            rospy.loginfo("Service client verbonden met /get_detection_info")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def get_detection_info(self):
        try:
            request = cameraRequest()  # Leeg request
            response = self.client(request)
            rospy.loginfo("Ontvangen gegevens:")
            rospy.loginfo("X: %.3f m", response.X)
            rospy.loginfo("Y: %.3f m", response.Y)
            rospy.loginfo("Z: %.3f m", response.Z)
            rospy.loginfo("Angle: %.1f graden", response.angle)
            rospy.loginfo("Object class: %s", response.object_class)
            return {
                "X": response.X,
                "Y": response.Y,
                "Z": response.Z,
                "angle": response.angle,
                "object_class": response.object_class
            }
        except rospy.ServiceException as e:
            rospy.logerr("Service-aanroep mislukt: %s", e)
            return None

if __name__ == '__main__':
    rospy.init_node('mondhygiene_client_node', anonymous=True)
    client = MondhygieneClient()
    info = client.get_detection_info()

