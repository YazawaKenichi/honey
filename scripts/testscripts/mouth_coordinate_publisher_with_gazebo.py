#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import argparse

import cv2 as cv
import numpy as np
import mediapipe as mp
import statistics
from std_msgs.msg import Float32MultiArray
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# publisher と subscriber を作成する
sub = ""
pub = ""
X_MAX = 0
Y_MAX = 0

def center(points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])

# 二点間のノルム
def norm(a, b):
    return np.linalg.norm(np.subtract(a, b), ord = 2)

def init():
    global sub, pub
    # カメラからサブスクライブ
    sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)
    # 口の座標のパブリッシュ
    pub = rospy.Publisher('/MouthCoordinate', Float32MultiArray, queue_size = 2)

def cca(point_):
    global X_MAX, Y_MAX
    x_ =   (point_[0] / X_MAX - 1 / 2) * 2
    y_ = - (point_[1] / Y_MAX - 1 / 2) * 2
    return [x_, y_]

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, default = 0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)
    parser.add_argument("--max_num_faces", type=int, default=1)
    parser.add_argument("--min_detection_confidence", help='min_detection_confidence', type=float, default=0.7)
    parser.add_argument("--min_tracking_confidence", help='min_tracking_confidence', type=int, default=0.5)
    args = parser.parse_args()
    return args

def callback(image_msg):
    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    global X_MAX, Y_MAX
    X_MAX = cap_width
    Y_MAX = cap_height
    image_center = [X_MAX / 2, Y_MAX / 2]

    max_num_faces = args.max_num_faces
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    # モデルロード #############################################################
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh( max_num_faces=max_num_faces, min_detection_confidence=min_detection_confidence, min_tracking_confidence=min_tracking_confidence)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding = "passthrough")
    results = face_mesh.process(image)

    global pub
    if results.multi_face_landmarks is not None:
        for face_landmarks in results.multi_face_landmarks:
            # 口の特徴点を作成
            coordinate = mouth_coordinate(debug_image, face_landmarks)
            pub.publish(Float32MultiArray(coordinate))
    else:
        print("lost...")

def mouth_coordinate(image, landmarks):
    landmark_point = []

    # 口の特徴点だけを抽出
    for _, landmark in enumerate(landmarks.landmark):
        landmark_point.append((landmark_x, landmark_y))

    # 口の特徴点を格納
    mouth_points = np.array([landmark_point[308] ,  landmark_point[415] ,  landmark_point[310] ,  landmark_point[311] ,  landmark_point[312] ,  landmark_point[13] ,  landmark_point[82] ,  landmark_point[81] ,  landmark_point[80] ,  landmark_point[191] ,  landmark_point[78] ,  landmark_point[95] ,  landmark_point[88] ,  landmark_point[178] ,  landmark_point[87] ,  landmark_point[14] ,  landmark_point[317] ,  landmark_point[402] ,  landmark_point[318] ,  landmark_point[324]])

    # 口の中心に点を打つ
    mouth_center = center(mouth_points)
    cv.circle(image, mouth_center, 2, (0, 0, 255), 2)

    mouth_center = cca(mouth_center)

    for value in mouth_center:
        print(format(value, '.3f') + ", ", end = "")
    print("")

if __name__ == '__main__':
    rospy.init_node("mouth_publisher_with_gazebo")
    init()

    while not rospy.is_shutdown():
        rospy.sleep(1)

