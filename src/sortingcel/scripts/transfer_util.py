#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

class TransformUtil:
    def __init__(self, source_frame="depthai_rgb_camera_optical_frame", target_frame="world"):
        self.source_frame = source_frame
        self.target_frame = target_frame

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform(self, x, y, z, qx, qy, qz, qw, timeout=5.0):
        """
        Zet een pose van de camera naar het wereldframe om.
        Geeft getransformeerde x, y, z en een Quaternion-object terug.
        """
        try:
            # Maak PoseStamped aan in source_frame
            pose_in = PoseStamped()
            pose_in.header.frame_id = self.source_frame
            pose_in.header.stamp = rospy.Time.now()
            pose_in.pose.position.x = x
            pose_in.pose.position.y = y
            pose_in.pose.position.z = z
            pose_in.pose.orientation.x = qx
            pose_in.pose.orientation.y = qy
            pose_in.pose.orientation.z = qz
            pose_in.pose.orientation.w = qw

            # Wacht op transformatie
            self.tf_buffer.can_transform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(timeout))
            pose_out = self.tf_buffer.transform(pose_in, self.target_frame)

            # Retourneer getransformeerde waarden
            pos = pose_out.pose.position
            ori = pose_out.pose.orientation
            return pos.x, pos.y, pos.z, ori  # ori is Quaternion
        except Exception as e:
            rospy.logerr("Transformatie mislukt: %s", str(e))
            return None

