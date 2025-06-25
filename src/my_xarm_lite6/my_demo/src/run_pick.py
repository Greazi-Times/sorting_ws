#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from tf.transformations import quaternion_from_euler
from transfer_util import TransformUtil
from robot_handler import RobotHandler

if __name__ == '__main__':
    rospy.init_node('pick_test_client')

    handler = RobotHandler()
    transformer = TransformUtil()

    # === Camera-input (in optical frame) ===
    x = -0.00054
    y = 0.0045
    z = 0.5382
    roll_deg = 0
    pitch_deg = 0
    yaw_deg = 113.6
    object_type = "normaal"

    # === Bereken quaternion van eulerhoek ===
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    # === Transformeer naar 'world' frame ===
    rospy.loginfo("Wachten op TF-transformatie...")
    result = transformer.transform(x, y, z, qx, qy, qz, qw)

    if not result:
        rospy.logerr("Kon transformatie niet uitvoeren.")
        exit(1)

    x_w, y_w, z_w, orientation = result

    # === Pick & place uitvoeren ===
    handler.sort(x_w, y_w, z_w, orientation, object_type)

