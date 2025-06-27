#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from depthai_ros_msgs.srv import camera, cameraRequest

class DetectionInfo:
    def __init__(self, x, y, z, qx, qy, qz, qw, angle, object_class, confidence):
        self.X = x
        self.Y = y
        self.Z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.angle = angle
        self.object_class = object_class
        self.confidence = confidence

    def __repr__(self):
        return ("DetectionInfo(X=%.3f, Y=%.3f, Z=%.1f, "
                "Quaternion=(%.3f, %.3f, %.3f, %.3f), "
                "angle=%.1f°, object_class='%s', confidence=%.1f%%)" % (
                    self.X, self.Y, self.Z,
                    self.qx, self.qy, self.qz, self.qw,
                    self.angle, self.object_class, self.confidence))


class CameraHandler:
    def __init__(self):
        self.client = None
        self.service_name = '/get_detection_info'

    def _ensure_client(self):
        if self.client is None:
            try:
                rospy.wait_for_service(self.service_name, timeout=5)
                self.client = rospy.ServiceProxy(self.service_name, camera)
                rospy.loginfo("CameraHandler: Connected to service %s", self.service_name)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("CameraHandler: Failed to connect to service: %s", str(e))
                self.client = None

    def check(self):
        self._ensure_client()
        if self.client is None:
            return None

        try:
            request = cameraRequest()  # Empty request
            response = self.client(request)

            pos = response.pose.pose.position
            ori = response.pose.pose.orientation

            rospy.loginfo("Detection info received:\n"
                          "  Position: X=%.3f, Y=%.3f, Z=%.1f\n"
                          "  Orientation (quat): x=%.3f y=%.3f z=%.3f w=%.3f\n"
                          "  Angle: %.1f°\n"
                          "  Object Class: %s\n"
                          "  Confidence: %.1f%%",
                          pos.x, pos.y, pos.z,
                          ori.x, ori.y, ori.z, ori.w,
                          response.angle,
                          response.object_class,
                          response.confidence)

            return DetectionInfo(
                x=pos.x,
                y=pos.y,
                z=pos.z,
                qx=ori.x,
                qy=ori.y,
                qz=ori.z,
                qw=ori.w,
                angle=response.angle,
                object_class=response.object_class,
                confidence=response.confidence
            )
        except rospy.ServiceException as e:
            rospy.logerr("CameraHandler: Service call failed: %s", str(e))
            return None
