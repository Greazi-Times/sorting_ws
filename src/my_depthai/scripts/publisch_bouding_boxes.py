#!/usr/bin/env python
'''
    circle_detector.py
    Purpose: displays boxes around detected objects
    @author Gerard Harkema
    @version 0.9 2023/01/05
    License: CC BY-NC-SA
'''

from __future__ import print_function
from depthai_ros_msgs.srv import camera, cameraResponse

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from depthai_ros_msgs.msg import SpatialDetectionArray
import random
import json


class detection_displayer:

    def handle_get_info(self, req):
        return cameraResponse(
            A=self.latest_info["A"],
            B=self.latest_info["B"],
            angle=self.latest_info["angle"],
            object_class=self.latest_info["object_class"]
        )

    def __init__(self, config_file):
        rospy.loginfo(config_file)
        self.display_image = False

        self.latest_info = {"A": 0.0, "B": 0.0, "angle": 0.0, "object_class": ""}
        rospy.Service('/get_detection_info', camera, self.handle_get_info)

        self.colors = []
        colors = []
        with open(config_file, 'r') as f:
            self.model_objects = json.loads(f.read())
            try:
                self.class_names = self.model_objects["class_names"]
                colors = self.model_objects["colors"]
                for color in colors:
                    self.colors.append(colors[color])
            except:
                self.class_names = self.model_objects["mappings"]["labels"]
                for class_name in self.class_names:
                    self.colors.append("#" + ''.join([random.choice('0123456789ABCDEF') for j in range(6)]))

        self.image_pub = rospy.Publisher("/image_out", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_in", Image, self.image_callback)
        self.detections_sub = rospy.Subscriber("/detections", SpatialDetectionArray, self.detections_callback)

        self.image_received = False

    def detections_callback(self, detections_data):

        if self.image_received:
            for detection in detections_data.detections:
                margin = 20  # pixels marge rondom de detectiebox
                x1 = max(0, int(detection.bbox.center.x - (detection.bbox.size_x / 2)) - margin)
                y1 = max(0, int(detection.bbox.center.y - (detection.bbox.size_y / 2)) - margin)
                x2 = min(self.image.shape[1] - 1, int(detection.bbox.center.x + (detection.bbox.size_x / 2)) + margin)
                y2 = min(self.image.shape[0] - 1, int(detection.bbox.center.y + (detection.bbox.size_y / 2)) + margin)


                x = detection.position.x
                y = detection.position.y
                z = detection.position.z

                center_x = detection.bbox.center.x
                center_y = detection.bbox.center.y

                fx = 1019.36
                fy = 1019.36
                cx = 653.54
                cy = 355.41

                A = (center_x - cx) * z / fx
                B = (center_y - cy) * z / fy

                class_index = detection.results[0].id
                class_color = self.colors[int(class_index)].strip("#")
                class_color = tuple(int(class_color[i:i + 2], 16) for i in (0, 2, 4))

                cv2.rectangle(self.image, (int(x1), int(y1)), (int(x2), int(y2)), class_color, 2)
                cv2.circle(self.image, (int(center_x), int(center_y)), 5, class_color, -1)

                h, w = self.image.shape[:2]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w - 1, x2), min(h - 1, y2)

                roi = self.image[y1:y2, x1:x2]

                if roi.size == 0:
                    rospy.logwarn("Lege ROI, skipping...")
                    continue

               # Stap 1: detecteer witte delen (kopje)
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh_white = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                # Stap 2: detecteer rode delen in HSV
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower_red1 = np.array([0, 100, 70])
                upper_red1 = np.array([10, 255, 255])
                lower_red2 = np.array([160, 100, 70])
                upper_red2 = np.array([180, 255, 255])

                mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
                mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)

                # Contouren van witte kop (grijswaarden)
                _, contours_white, _ = cv2.findContours(thresh_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Contouren van rode steel (HSV-mask)
                _, contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Combineer contourlijsten
                contours = contours_white + contours_red

                if len(contours) == 0:
                    angle = 0
                else:
                    largest_contour = max(contours, key=cv2.contourArea)
                    
                    for contour in contours:
                        if cv2.contourArea(contour) < 100:
                            continue
                        offset_contour = contour + np.array([[x1, y1]])
                        cv2.drawContours(self.image, [offset_contour], -1, (0, 255, 0), 2)


                    rect = cv2.minAreaRect(largest_contour)
                    (center_box, size_box, angle) = rect

                    width, height = size_box

                    # Alleen als contour langwerpig is, bereken we een zinvolle hoek
                    if width > 0 and height > 0:
                        aspect_ratio = max(width, height) / min(width, height)
                        if width > 0 and height > 0:
                            aspect_ratio = max(width, height) / min(width, height)
    
                        if aspect_ratio > 1.5:
                            if width < height:
                                angle += 90
                        elif 0.9 <= aspect_ratio <= 1.1:
                            data_pts = np.reshape(largest_contour, (-1, 2)).astype(np.float64)
                            if data_pts.shape[0] >= 2:
                                mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)[:2]
                                angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0]) * 180 / np.pi
                        else:
                            angle = 0


                    else:
                        angle = 0


                text = '%s: %.2f%%' % (self.class_names[detection.results[0].id], detection.results[0].score * 100)
                image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)

                #text = 'x: %.3f m' % x
                #image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)

                #text = 'y: %.3f m' % y
                #image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) + 45), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)

                #text = 'z: %.3f m' % z
                #image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) + 70), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 2, cv2.LINE_AA)

                text = 'A: %.3f m' % A
                image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) + 20), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 4, cv2.LINE_AA)

                text = 'B: %.3f m' % B
                image = cv2.putText(self.image, text, (int(x1) + 5, int(y2) + 45), cv2.FONT_HERSHEY_SIMPLEX, 1, class_color, 4, cv2.LINE_AA)

                text = 'Angle: %.1f deg' % angle
                image = cv2.putText(self.image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 4)

                self.latest_info["A"] = A
                self.latest_info["B"] = B
                self.latest_info["angle"] = angle
                self.latest_info["object_class"] = self.class_names[detection.results[0].id]

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
            except CvBridgeError as e:
                print(e)

            self.image_received = False

    def image_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True

        if self.display_image:
            cv2.imshow("Detections window", self.image)
            cv2.waitKey(3)


def main(args):
    rospy.init_node('publisch_bouding_boxes', anonymous=True)
    node_name = rospy.get_name()

    nnConfig = rospy.get_param(node_name + '/nnConfig')
    resourceBaseFolder = rospy.get_param(node_name + '/resourceBaseFolder')

    ic = detection_displayer(resourceBaseFolder + '/' + nnConfig)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
