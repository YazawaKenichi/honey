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

i = 0

def center(points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])

def callback(data):
    bridge = CvBridge()
    cv2_array = bridge.imgmsg_to_cv2(data)
    main(cv2_array)

def listener():
    rospy.init_node('mouth_depth', anonymous = True)
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback = callback, queue_size = 10)
    rospy.spin()

def main(image):
    global i
    print(str(i) + " write")
    cv2.imwrite("../depth_images/" + str(i).zfill(6) + ".png", image)
    i += 1

if __name__ == '__main__':
    if not os.path.exists("../depth_images/"):
        os.mkdir('../depth_images/')
    listener()

