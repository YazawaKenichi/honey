#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import argparse

import rospy
import cv2 as cv
import numpy as np
import mediapipe as mp
from collections import deque
from std_msgs.msg import String
from geometry_msgs.msg import Point

diff_point = Point() # 偏差
target_point = Point()  # カメラ画像中心の座標
target_point.z = 0      # camera target z

# 二点間の距離を図るノルム２
def norm():
        return np.linalg.norm(np.subtract(a, b), ord = 2)

class CvFpsCalc(object):
    def __init__(self, buffer_len=1):
        self._start_tick = cv.getTickCount()
        self._freq = 1000.0 / cv.getTickFrequency()
        self._difftimes = deque(maxlen=buffer_len)

    def get(self):
        current_tick = cv.getTickCount()
        different_time = (current_tick - self._start_tick) * self._freq
        self._start_tick = current_tick

        self._difftimes.append(different_time)

        fps = 1000.0 / (sum(self._difftimes) / len(self._difftimes))
        fps_rounded = round(fps, 2)

        return fps_rounded


def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)    # Real sens video6
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

    rospy.init_node("publish_mediapipe")   # node 初期化
    #pub_str = rospy.Publisher("mediapipe_string", String, queue_size=10)    # publisher 作成
    pub_diff = rospy.Publisher("mediapipe_difference", Point, queue_size=10)    # publisher 作成


    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    max_num_faces = args.max_num_faces
    #print('max_num_faces = ', end='')
    #print(max_num_faces)
    min_detection_confidence = args.min_detection_confidence
    #print('min_detection_confidence = ', end='')
    #print(min_detection_confidence)
    min_tracking_confidence = args.min_tracking_confidence
    #print('min_tracking_confidence = ', end='')
    #print(min_tracking_confidence)

    use_brect = args.use_brect

    # カメラ準備 ###############################################################
    cap = cv.VideoCapture(cap_device)
    print('Capture_device : ', end='')
    print(cap_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    print('WIDTH : x_max = ', end='')
    x_max = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    print(x_max)
    #print(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)
    print('HEIGHT : y_max = ', end='')
    y_max = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
    print(y_max)
    #print(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    # cap.set() は実質反映されていない。
    
    target_point.x = (x_max/2)
    target_point.y = (y_max/2)

    # モデルロード #############################################################
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=max_num_faces,
        min_detection_confidence=min_detection_confidence,
        min_tracking_confidence=min_tracking_confidence,
    )

    # FPS計測モジュール ########################################################
    cvFpsCalc = CvFpsCalc(buffer_len=10) # 10.31 ここまで終わった
    #print(cvFpsCalc.get())
    #print(cvFpsCalc.get())

    ret, image = cap.read()
    print('次元数 :', image.ndim)
    print('各次元の長さ : (y, x, ?) =', image.shape)
    print('全要素数 :', image.size)

    #rate = rospy.Rate(10)

    #while not rospy.is_shutdown():
    while True:
        #print(cvFpsCalc.get()) #whileないだけのようだ。↓　
        display_fps = cvFpsCalc.get()
        #print(display_fps)
        #print(cvFpsCalc.get()) #この文を加えるたび、fpsが + display_fps される。
        #print(cvFpsCalc.get())
        #print(cvFpsCalc.get())

        # カメラキャプチャ #####################################################
        #print(cap.read())
        ret, image = cap.read() #この書き方で成立する意味がわからん。
        #print(cap.get())
        #print(ret)
        #print(image)
        if not ret:
            break
        image = cv.flip(image, 1)  # ミラー表示
        debug_image = copy.deepcopy(image)
        #print(debug_image)
        #print('/////////////////////////////////////////////////////////////////////////??')


        # 検出実施 #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        results = face_mesh.process(image)

        # 描画 ################################################################
        #print(results.multi_face_landmarks)
        #print('/////////////////////////////////////////////////////////////////////////??')
        if results.multi_face_landmarks is not None:
            for face_landmarks in results.multi_face_landmarks:
                # 外接矩形の計算
                brect = calc_bounding_rect(debug_image, face_landmarks)
                # 描画
                debug_image = draw_landmarks(debug_image, face_landmarks)
                debug_image = draw_bounding_rect(use_brect, debug_image, brect)

        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)


        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        cv.imshow('MediaPipe Face Mesh Demo', debug_image)



        # 1 秒ごとにトピックを送信
        #rate = rospy.Rate(1)
        #while not rospy.is_shutdown():

        # トピックを送信
        #msg_str = "Publishing {}".format(rospy.get_time())
        #pub_str.publish(msg_str)
        # これ無いとだめじゃね？
        global diff_point
        pub_diff.publish(diff_point)

        #rospy.loginfo("Message '{}' published".format(msg_str))
        rospy.loginfo(diff_point)

        # 1 秒スリープする
        #rate.sleep()



    cap.release()
    cv.destroyAllWindows()

def calc_bounding_rect(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]
    #print('image.shape =', image.shape)
    #print('/////////////////////////////////////////////////////////////////////////??')

    landmark_array = np.empty((0, 2), int)

    for _, landmark in enumerate(landmarks.landmark):
        #print(landmarks.landmark)
        #print('/////////////////////////////////////////////////////////////////////////??')
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)


        landmark_point = [np.array((landmark_x, landmark_y))]

        landmark_array = np.append(landmark_array, landmark_point, axis=0)

    x, y, w, h = cv.boundingRect(landmark_array)

    return [x, y, x + w, y + h]


def draw_landmarks(image, landmarks):
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_point = []
    landmark_pointz =[]
    depth = []  # Yazawa

    for index, landmark in enumerate(landmarks.landmark):
        if landmark.visibility < 0 or landmark.presence < 0:
            continue

        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)
        landmark_z = landmark.z
        #print(landmark_z)
        #print((landmark_x, landmark_y))
        #print('/////////////////////////////////////////////////////////////////////////??')

        landmark_point.append((landmark_x, landmark_y))
        landmark_pointz.append((landmark_x, landmark_y, landmark_z))
        #print(landmark_point)
        #print('/////////////////////////////////////////////////////////////////////////??')

        cv.circle(image, (landmark_x, landmark_y), 2, (0, 0, 255), 1) # 点の正体

        cv.circle(image, (landmark_x, landmark_y), 1, (0, 255, 0), 1)

        # 深さ Yazawa
        depth.append(landmark.z)

    if len(landmark_point) > 0:
        # 参考：https://github.com/tensorflow/tfjs-models/blob/master/facemesh/mesh_map.jpg

        # 左眉毛(55：内側、46：外側)
        cv.line(image, landmark_point[55], landmark_point[65], (0, 255, 0), 2)
        cv.line(image, landmark_point[65], landmark_point[52], (0, 255, 0), 2)
        cv.line(image, landmark_point[52], landmark_point[53], (0, 255, 0), 2)
        cv.line(image, landmark_point[53], landmark_point[46], (0, 255, 0), 2)

        # 右眉毛(285：内側、276：外側)
        cv.line(image, landmark_point[285], landmark_point[295], (0, 255, 0),
                2)
        cv.line(image, landmark_point[295], landmark_point[282], (0, 255, 0),
                2)
        cv.line(image, landmark_point[282], landmark_point[283], (0, 255, 0),
                2)
        cv.line(image, landmark_point[283], landmark_point[276], (0, 255, 0),
                2)

        # 左目 (133：目頭、246：目尻)
        cv.line(image, landmark_point[133], landmark_point[173], (0, 255, 0),
                2)
        cv.line(image, landmark_point[173], landmark_point[157], (0, 255, 0),
                2)
        cv.line(image, landmark_point[157], landmark_point[158], (0, 255, 0),
                2)
        cv.line(image, landmark_point[158], landmark_point[159], (0, 255, 0),
                2)
        cv.line(image, landmark_point[159], landmark_point[160], (0, 255, 0),
                2)
        cv.line(image, landmark_point[160], landmark_point[161], (0, 255, 0),
                2)
        cv.line(image, landmark_point[161], landmark_point[246], (0, 255, 0),
                2)

        cv.line(image, landmark_point[246], landmark_point[163], (0, 255, 0),
                2)
        cv.line(image, landmark_point[163], landmark_point[144], (0, 255, 0),
                2)
        cv.line(image, landmark_point[144], landmark_point[145], (0, 255, 0),
                2)
        cv.line(image, landmark_point[145], landmark_point[153], (0, 255, 0),
                2)
        cv.line(image, landmark_point[153], landmark_point[154], (0, 255, 0),
                2)
        cv.line(image, landmark_point[154], landmark_point[155], (0, 255, 0),
                2)
        cv.line(image, landmark_point[155], landmark_point[133], (0, 255, 0),
                2)

        # 右目 (362：目頭、466：目尻)
        cv.line(image, landmark_point[362], landmark_point[398], (0, 255, 0),
                2)
        cv.line(image, landmark_point[398], landmark_point[384], (0, 255, 0),
                2)
        cv.line(image, landmark_point[384], landmark_point[385], (0, 255, 0),
                2)
        cv.line(image, landmark_point[385], landmark_point[386], (0, 255, 0),
                2)
        cv.line(image, landmark_point[386], landmark_point[387], (0, 255, 0),
                2)
        cv.line(image, landmark_point[387], landmark_point[388], (0, 255, 0),
                2)
        cv.line(image, landmark_point[388], landmark_point[466], (0, 255, 0),
                2)

        cv.line(image, landmark_point[466], landmark_point[390], (0, 255, 0),
                2)
        cv.line(image, landmark_point[390], landmark_point[373], (0, 255, 0),
                2)
        cv.line(image, landmark_point[373], landmark_point[374], (0, 255, 0),
                2)
        cv.line(image, landmark_point[374], landmark_point[380], (0, 255, 0),
                2)
        cv.line(image, landmark_point[380], landmark_point[381], (0, 255, 0),
                2)
        cv.line(image, landmark_point[381], landmark_point[382], (0, 255, 0),
                2)
        cv.line(image, landmark_point[382], landmark_point[362], (0, 255, 0),
                2)

        # 口 (308：右端、78：左端)

        # 右上（見る側にとって）
        cv.line(image, landmark_point[308], landmark_point[415], (0, 255, 0), 2)
        cv.line(image, landmark_point[415], landmark_point[310], (0, 255, 0), 2)
        cv.line(image, landmark_point[310], landmark_point[311], (0, 255, 0), 2)
        cv.line(image, landmark_point[311], landmark_point[312], (0, 255, 0), 2)
        cv.line(image, landmark_point[312], landmark_point[13], (0, 255, 0), 2)
        #cv.circle(image, landmark_point[13], 3, (255, 0, 0), 2)
        #print('landmark_point[13] = ', end='')
        #print(landmark_point[13])

        # 左上
        cv.line(image, landmark_point[13], landmark_point[82], (0, 255, 0), 2)
        cv.line(image, landmark_point[82], landmark_point[81], (0, 255, 0), 2)
        cv.line(image, landmark_point[81], landmark_point[80], (0, 255, 0), 2)
        cv.line(image, landmark_point[80], landmark_point[191], (0, 255, 0), 2)
        cv.line(image, landmark_point[191], landmark_point[78], (0, 255, 0), 2)

        # 左下
        cv.line(image, landmark_point[78], landmark_point[95], (0, 255, 0), 2)
        cv.line(image, landmark_point[95], landmark_point[88], (0, 255, 0), 2)
        cv.line(image, landmark_point[88], landmark_point[178], (0, 255, 0), 2)
        cv.line(image, landmark_point[178], landmark_point[87], (0, 255, 0), 2)
        cv.line(image, landmark_point[87], landmark_point[14], (0, 255, 0), 2)
        #cv.circle(image, landmark_point[14], 3, (0, 0, 255), 2)
        #print('landmark_point[14] = ', end='')
        #print(landmark_point[14])

        #print('[ x, y ] : [14] - [13] = ', end='')
        diff = [ (a - b)/2.0 for (a, b) in zip(landmark_point[14], landmark_point[13]) ]
        #print(diff)

        #print('[ →  , ↓  ] : target = ', end='')
        summ = [ a + b for (a, b) in zip(landmark_point[13], diff) ]
        target = [ int(i) for i in summ ]   # 口中心の座標所得
        #print(target)

        # 深さの計算 Yazawa
        size = int(len(depth))
        average_depth = sum(depth) / size
        mouth_depth = 7 - (float(average_depth) * 1e+05)

        current_point = Point() # 口中心の座標
        current_point.x = target[0]
        current_point.y = target[1]
        current_point.z = mouth_depth # 深さ情報の追加 Yazawa

        # これ無いとだめじゃね？
        global diff_point
        diff_point.y = ( target_point.y - current_point.y )
        diff_point.z = ( target_point.z - current_point.z )

        cv.circle(image, target, 3, (255, 0, 255), 2) # 口中心の描写

        # 右下
        cv.line(image, landmark_point[14], landmark_point[317], (0, 255, 0), 2)
        cv.line(image, landmark_point[317], landmark_point[402], (0, 255, 0), 2)
        cv.line(image, landmark_point[402], landmark_point[318], (0, 255, 0), 2)
        cv.line(image, landmark_point[318], landmark_point[324], (0, 255, 0), 2)
        cv.line(image, landmark_point[324], landmark_point[308], (0, 255, 0), 2)

        #cv.putText(image, "z:" + str(round(landmark_z, 3)), (landmark_x - 10, landmark_y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv.LINE_AA)

    return image


def draw_bounding_rect(use_brect, image, brect):
    if use_brect:
        # 外接矩形
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 255, 0), 2)

    return image

if __name__ == '__main__':
    print(diff_point)
    main()
