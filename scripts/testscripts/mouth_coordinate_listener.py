#!/usr/bin/env python3
# coding: utf-8

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import cv2
import os

def callback(data_):
    tmp = data_.data
    print(tmp)

def listener():
    rospy.init_node('mouoth_coordinate_listener', anonymous = True)
    rospy.Subscriber("/MouthCoordinate", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

