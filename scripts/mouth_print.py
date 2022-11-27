#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import copy
import rospy
import argparse
import statistics
import cv2 as cv
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
    global i
    if i >= 150:
        i = 0
    else:
        bridge = CvBridge()
        cv2_array = bridge.imgmsg_to_cv2(data)
        mediapipe(cv2_array)
        i = i + 1

def listener():
    rospy.init_node('hogehoge_listener', anonymous = True)
    rospy.Subscriber("/camera/fake_cam_color/image_raw", Image, callback)
    rospy.spin()

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)

    parser.add_argument("--max_num_faces", type=int, default=1)
    parser.add_argument("--min_detection_confidence",
                        help='min_detection_confidence',
                        type=float,
                        default=0.7)
    parser.add_argument("--min_tracking_confidence",
                        help='min_tracking_confidence',
                        type=int,
                        default=0.5)

    parser.add_argument('--use_brect', action='store_true')

    args = parser.parse_args()

    return args


def mediapipe(cv2_array):
    # 引数解析 #################################################################
    args = get_args()

    max_num_faces = args.max_num_faces
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    use_brect = args.use_brect

    # モデルロード #############################################################
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(max_num_faces=max_num_faces, min_detection_confidence=min_detection_confidence, min_tracking_confidence=min_tracking_confidence)

    # カメラキャプチャ #####################################################
    image = cv2_array
    debug_image = copy.deepcopy(image)

    # 検出実施 #############################################################
    image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    results = face_mesh.process(image)

    # 描画 ################################################################
    if results.multi_face_landmarks is not None:
        for face_landmarks in results.multi_face_landmarks:
            # 描画
            debug_image = draw_landmarks(debug_image, face_landmarks)
    else:
        print("lost...")

    # 画面反映 #############################################################
    # cv.imshow('MediaPipe Face Mesh Demo', debug_image)

def draw_landmarks(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    landmark_point = []

    for landmark in landmarks.landmark[13:15]:
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point.append((landmark_x, landmark_y))

    # 口の特徴点を格納
    mouth_points = np.array([landmark_point[0], landmark_point[1]])
    # 口の座標を標準出力
    print(center(mouth_points))

    return image

if __name__ == '__main__':
    if not os.path.exists("../images/"):
        os.mkdir('../images/')
    listener()

