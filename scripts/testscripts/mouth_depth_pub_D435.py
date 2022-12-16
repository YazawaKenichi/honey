#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import copy
import rospy
import argparse
import statistics
import cv2
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

def center(self, points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])


class Depth:
    def callback(self, data):
        cv2_array = self.bridge.imgmsg_to_cv2(data)
        self.main(cv2_array)

    def __init__(self):
        self.i = 0
        rospy.init_node('mouth_depth', anonymous = True)
        self.bridge = CvBridge()
        self.talker()
        self.listener()
        rospy.spin()

    def listener(self):
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback = self.callback, queue_size = 10)

    def talker(self):
        self.pub = rospy.Publisher("/depth", Float32, queue_size = 2)

    def main(self, image):
        depth_array = np.array(image, dtype = np.uint16) * 0.001
        height, width = image.shape[:3]
        # cv2.circle(depth_image, (int(240/2), int(240/2)), 2, (0, 0, 255), 2)
        # cv2.imshow("test", image)
        print(depth_array[int(width/2), int(height/2)])

if __name__ == '__main__':
    if not os.path.exists("../depth_images/"):
        os.mkdir('../depth_images/')
    Depth()

