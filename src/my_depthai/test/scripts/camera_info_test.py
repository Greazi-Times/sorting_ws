#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

def camera_info_callback(msg):
    rospy.loginfo("Camera Info ontvangen:")
    rospy.loginfo("Focal length (fx, fy): (%.2f, %.2f)", msg.K[0], msg.K[4])
    rospy.loginfo("Principal point (cx, cy): (%.2f, %.2f)", msg.K[2], msg.K[5])
    rospy.loginfo("Distortion model: %s", msg.distortion_model)
    rospy.loginfo("Image size: %dx%d", msg.width, msg.height)

def main():
    rospy.init_node('camera_info_test', anonymous=True)
    rospy.Subscriber("/oak/stereo/camera_info", CameraInfo, camera_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

