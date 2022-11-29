#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import copy
import rospy
import signal
import argparse
import statistics
import cv2 as cv
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

cont = True

def handler(signal, frame):
    global cont
    cont = False

def center(points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])

class MouthPub:
    def __init__(self):
        self.i = 0
        # Create Subscriber
        self._sub = rospy.Subscriber("/camera/fake_cam_color/image_raw", Image, self.callback)
        # Create Publisher
        self._pub = rospy.Publisher('/MouthCoordinate', Int16MultiArray, queue_size = 10)

    def get_args(self):
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

    def mediapipe(self, cv2_array):
        # 引数解析
        args = self.get_args()

        max_num_faces = args.max_num_faces
        min_detection_confidence = args.min_detection_confidence
        min_tracking_confidence = args.min_tracking_confidence

        use_brect = args.use_brect

        # モデルロード
        mp_face_mesh = mp.solutions.face_mesh
        face_mesh = mp_face_mesh.FaceMesh(max_num_faces=max_num_faces, min_detection_confidence=min_detection_confidence, min_tracking_confidence=min_tracking_confidence)

        # カメラキャプチャ
        image = cv2_array
        debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = face_mesh.process(image)

        if results.multi_face_landmarks is not None:
            for face_landmarks in results.multi_face_landmarks:
                self.draw_landmarks(debug_image, face_landmarks)
        else:
            print("lost...")

    def draw_landmarks(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]
        landmark_point = []

        for landmark in landmarks.landmark[13:15]:
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point.append((landmark_x, landmark_y))

        # 口の特徴点を格納
        mouth_points = np.array([landmark_point[0], landmark_point[1]])
        # 口の特徴点から中心座標を取得
        center_mouth_points = center(mouth_points)
        # 口の座標を標準出力
        print(center_mouth_points)
        # 口の座標をパブリッシュ
        self.publish(center_mouth_points)

    def callback(self, imagedata):
        if self.i >= 1:
            bridge = CvBridge()
            cv2_array = bridge.imgmsg_to_cv2(imagedata)
            self.mediapipe(cv2_array)
            self.i = 0
        else:
            self.i = self.i + 1

    def publish(self, pubdata):
        # 配列を Publish する
        # ndarray 型のままでは Publish できないため Int16MultiArray 型に変換してから Publish する必要がある
        publish_data = Int16MultiArray(data = pubdata)
        self._pub.publish(publish_data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--text', '-t', type=str, dest='text', help='set message to speech')
    parser.add_argument('--label', '-l', metavar='L', type=str, nargs='?', dest='label', help='set label for this message')
    parser.add_argument('-p', action='store_true', dest='periodic_flag', help='set either periodic message or not')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, handler)

    rospy.init_node('MouthNode', anonymous = True, disable_signals = True)

    node = MouthPub()
    rate = rospy.Rate(1)

    while cont:
        rate.sleep()

    rospy.signal_shutdown('finish')
    rospy.spin()

