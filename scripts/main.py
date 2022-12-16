#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import argparse

import cv2 as cv
import numpy as np
import mediapipe as mp
import statistics

def center(points):
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

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


def main():
    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    max_num_faces = args.max_num_faces
    min_detection_confidence = args.min_detection_confidence
    min_tracking_confidence = args.min_tracking_confidence

    use_brect = args.use_brect

    # カメラ準備 ###############################################################
    cap = cv.VideoCapture(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

    # モデルロード #############################################################
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=max_num_faces,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )


    while True:
        # カメラキャプチャ #####################################################
        ret, image = cap.read()
        if not ret:
            break
        image = cv.flip(image, 1)  # ミラー表示
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

        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        # cv.imshow('MediaPipe Face Mesh Demo', debug_image)

    cap.release()
    cv.destroyAllWindows()

def draw_landmarks(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    landmark_point = []

    # 口の特徴点だけを抽出
    for _, landmark in enumerate(landmarks.landmark[13:15]):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_point.append((landmark_x, landmark_y))
        depth = landmark.z


    # 上唇中央と下唇中央の座標を標準出力
    # 座標が二つしか格納されていないことを確認する
    # print(landmark_point)

    # 上唇の中央と下唇の中央に点を打つ
    # cv.circle(image, landmark_point[0], 2, (0, 255, 255), 2)
    # cv.circle(image, landmark_point[1], 2, (255, 0, 0), 2)

    # 口の特徴点を格納
    mouth_points = np.array([landmark_point[0], landmark_point[1]])

    # 口の中心に点を打つ
    cv.circle(image, center(mouth_points), 2, (0, 0, 255), 2)
    res = center(mouth_points)
    depth = (depth + float(5e-02)) * 100
    res = np.append(res, depth)
    print(res)

    return image

if __name__ == '__main__':
    main()

