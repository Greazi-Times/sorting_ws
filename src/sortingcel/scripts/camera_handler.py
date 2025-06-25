#!/usr/bin/env python

"""
    camera_handler.py
    Purpose: Service handler to interact with /get_detection_info and extract detection coordinates and type.
    Author: [Your Name]
    Version: 1.0
"""

import rospy
from depthai_ros_msgs.srv import camera, cameraRequest

class DetectionInfo:
    def __init__(self, x, y, z, angle, object_class):
        self.X = x
        self.Y = y
        self.Z = z
        self.angle = angle
        self.object_class = object_class

    def __repr__(self):
        return f"DetectionInfo(X={self.X}, Y={self.Y}, Z={self.Z}, angle={self.angle}, object_class='{self.object_class}')"


class CameraHandler:
    def __init__(self):
        rospy.wait_for_service('/get_detection_info')
        try:
            self.client = rospy.ServiceProxy('/get_detection_info', camera)
            rospy.loginfo("Service client connected to /get_detection_info")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to connect to service: %s", e)
            raise

    def check(self):
        """
        Calls the /get_detection_info service and returns DetectionInfo object.
        """
        try:
            request = cameraRequest()  # Empty request
            response = self.client(request)

            rospy.loginfo("Received detection info: X=%.3f, Y=%.3f, Z=%.3f, angle=%.1f, object_class=%s",
                          response.X, response.Y, response.Z, response.angle, response.object_class)

            return DetectionInfo(
                x=response.X,
                y=response.Y,
                z=response.Z,
                angle=response.angle,
                object_class=response.object_class
            )

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None


# Optional test when running as a script
if __name__ == '__main__':
    rospy.init_node('camera_handler_node', anonymous=True)
    handler = CameraHandler()
    detection = handler.check()
    if detection:
        print(detection)
