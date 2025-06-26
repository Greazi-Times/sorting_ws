#!/usr/bin/env python3

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
        return (f"DetectionInfo(X={self.X}, Y={self.Y}, Z={self.Z}, "
                f"angle={self.angle}, object_class='{self.object_class}')")

class CameraHandler:
    def __init__(self):
        self.client = None  # Client is not initialized yet
        self.service_name = '/get_detection_info'

    def _ensure_client(self):
        """
        Lazy initialization of the service client when first needed.
        """
        if self.client is None:
            try:
                rospy.wait_for_service(self.service_name, timeout=5)
                self.client = rospy.ServiceProxy(self.service_name, camera)
                rospy.loginfo("CameraHandler: Connected to service /get_detection_info.")
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("CameraHandler: Failed to connect to service: %s", e)
                self.client = None

    def check(self):
        """
        Calls the /get_detection_info service and returns a DetectionInfo object.
        """
        self._ensure_client()
        if self.client is None:
            rospy.logerr("CameraHandler: Cannot call detection service, client not available.")
            return None

        try:
            request = cameraRequest()  # Empty request
            response = self.client(request)

            rospy.loginfo("Detection info received: X=%.3f, Y=%.3f, Z=%.3f, angle=%.1f, object_class=%s",
                          response.X, response.Y, response.Z, response.angle, response.object_class)

            return DetectionInfo(
                x=response.X,
                y=response.Y,
                z=response.Z,
                angle=response.angle,
                object_class=response.object_class
            )
        except rospy.ServiceException as e:
            rospy.logerr("CameraHandler: Service call failed: %s", e)
            return None
