# ROS + Python Scripting Technique

## ROS Master との通信を始める
### 基本
```
rospy.init_node('NODE_NAME')
```
### 匿名化
ノード名が重複しないように rospy が固有のノード名を生成する
```
rospy.init_node('NODE_NAME', anonymouse = True)
```

## SubScribe する
```
rospy.Subscriber('TOPIC_NAME', MESSAGE_TYPE, CALLBACK)
```

|変数|説明
|---|---
|TOPIC_NAME|トピックの名前を入れる
|MESSAGE_TYPE|メッセージの型を指定する
|CALLBACK|メッセージをサブスクライブしたときに呼び出される関数（コールバック関数）を指定する

### トピックの情報を手に入れる
トピック名の一覧表示
```
rostopic list
```
トピックの型を表示
```
rostopic info TOPIC_NAME
```

## 自発的にノードが終了するのを防ぐ
コールバック関数の中で呼び出す参考
```
rospy.spin()
```

## あるトピックからメッセージをサブスクライブし、他のトピックにメッセージをパブリッシュする
```
#! /usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String

import time

class testNode():
    # 初期化
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber('topic name', String, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher('topic name', String, queue_size = 1)

    # Subscribe したら以下を実行
    def callback(self, data):
        # callback関数の処理をかく
        # data を publish する
        self.publish(data)

    # 好きなときに publish できるように関数化しておく
    def publish(self, data):
        # data を pub = rospy.Publisher() で決めた通りに publish する
        self.pub.publish(data)

    def function(self, data):
        # そのほかの処理もあったら書く
        return data

if __name__ == '__main__':
    rospy.init_node('test_node')

    time.sleep(3.0)
    node = testNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
```

### 参考
[【Python】ROSのプログラムをPythonのclassを使ったらとても便利だった (Qiita)](https://qiita.com/koichi_baseball/items/d15d52856188120647f4)

# ROS msgs.msg
## std_masgs.msg
### std_msgs.msg/Int16MultiArray
#### 任意の numpy.ndarray 型データを Publish する
##### 型変換
`numpy.ndarray` 型配列を Publish するときは `std_msgs.msg/Int16MultiArray` 型配列に変換する必要がある

型変換の方法は今まで見たこと無いかも？

今まで通りの方法で型変換するとエラーを吐く

以下の方法で型変換ができる

```
data_pub = Int16MultiArray(data = data_ndarray)
```

##### クラスのインスタンス化
```
publisher = rospy.Publisher(TOPICNAME, Int16MultiArray, queue_size = 10)
```

`queue_size` はデータがロストしないようなサイズを指定する必要がある

`queue_size = 1` が何バイトのキューなのかわからんけど...

#### 参考
[std_msgs/Int16MultiArray Message](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int16MultiArray.html)

## sensor_msgs.msg
### sensor_msgs.msg/Image
#### Image 型メッセージを OpenCV で読む

下記の式は、トピック '/camera/fake_cam_color/image_raw' にパブリッシュされた 'sensor_msgs.msg/Image' 型の画像メッセージを、OpenCV で処理可能な 'np.ndarray' 型の画像クラスに変換する。

```
cv2_array = CvBridge().imgmsg_to_cv2(ros_array)
```

それを受けて試しに作成したコードが以下

```
import os
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image

# 型変換のためのインポート
from cv_bridge import CvBridge

i = 0

# サブスクライブしたら data にメッセージが入り、以下の関数を実行
def callback(data):
    global i
    print(i)
    try:
        # データ変換クラスのインスタンス化
        bridge = CvBridge()
        # 実際にデータ変換を行う
        cv2_array = bridge.imgmsg_to_cv2(data)
        # サブスクライブした画像を '../images/******.png' に保存
        # str(i).zfill(6) : 数字が 6 桁になるようにゼロ埋め
        cv2.imwrite("../images/" + str(i).zfill(6) + ".png", cv2_array)
        # rospy.loginfo(cv2_array)
    except Exception as err:
        rospy.logerr(err)
    if i >= 300:
        i = 0
    else:
        i = i + 1

def listener():
    rospy.init_node('image_listener_test.py', anonymous = True)
    rospy.Subscriber("/camera/fake_cam_color/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    if not os.path.exists("../images/"):
        os.mkdir('../images/')
    listener()

```

全体的な流れ

1. ノードを立ち上げる
1. トピックからメッセージをサブスクライブする
1. サブスクライブされたら 'Image' 型から 'ndarray' 型に変換する
1. 変換した画像を保存する
1. 300 枚保存したら 0 枚目から上書きしていく

つまり最終的に、センサからの動画が連続画像として保存される。

これを動画に変換することで動画を作成する。

> [この方法を取る理由] はじめは 'cv2.imread()' を用いれば、OpenCV で画像表示ができると考え、'cv2.imwrite()' を 'cv2.imread()' でやっていた。しかし実際にやってみると、描画速度の問題なのか、OpenCV の内部処理的な問題なのか、重くて固まるし、肝心のウィンドウは真っ暗なため、リアルタイムで閲覧することはできないと判断し、とりあえず、画像を保存する手段を取った。

### 参考
[Converting between ROS images (Python)](https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)

## rospy + signal
### [Ctrl] + [C] できれいに終わるようにする

```
#!/usr/bin/env python3
# coding: utf-8

import signal
import rospy

cont = True

def handler(signal, frame):
    global cont
    cont = False

def the_node():
    global cont

    rospy.init_node('the_node', disable_signals = True)

    rate = rospy.Rate(40)
    while cont:

        # main process

        rate.sleep()

    rospy.signal_shutdown('finish')
    rospy.spin()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    the_node()
```

#### 参考
[[rospy] ctrl-cで綺麗に終わるようにする - Qiita](https://qiita.com/idev_jp/items/bade8361301b0c1db417)

## perser
### python コード実行時に引数処理をする

```
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--text', '-t', type=str, dest='text', help='set message to speech')
    parser.add_argument('--label', '-l', metavar='L', type=str, nargs='?', dest='label', help='set label for this message')
    parser.add_argument('-p', action='store_true', dest='periodic_flag', help='set either periodic message or not')
    args = parser.parse_args()

    rospy.init_node( ...
```

これ記述しないと `launch` ファイルで `<node>` 指定してスクリプト実行するときにエラー吐きまくる

具体的なエラーはたとえば以下のようなもの（あくまで例）

```
process[makeSoundRequest-1]: started with pid [11925]
usage: makeSoundRequest.py [-h] [--text TEXT] [--label [L]] [-p]
makeSoundRequest.py: error: unrecognized arguments: __name:=makeSoundRequest __log:=/home/nvidia/.ros/log/0681ca48-10f0-11ea-8929-00044bc771c7/makeSoundRequest-1.log
[makeSoundRequest-1] process has died [pid 11925, exit code 2, cmd /home/nvidia/catkin_ws/src/sound_indication/scripts/makeSoundRequest.py --text please say something --label mode1 __name:=makeSoundRequest __log:=/home/nvidia/.ros/log/0681ca48-10f0-11ea-8929-00044bc771c7/makeSoundRequest-1.log].
log file: /home/nvidia/.ros/log/0681ca48-10f0-11ea-8929-00044bc771c7/makeSoundRequest-1*.log
all processes on machine have died, roslaunch will exit
```

ポイントは `---.py: error: unrecognized arguments: __name:=--- __log:=/home/---/.ros/log/--------------/---------.log` の部分

#### 参考
[[easy] launch file syntax: how to pass quotation mark to a python script](https://answers.ros.org/question/338617/easy-launch-file-syntax-how-to-pass-quotation-mark-to-a-python-script/)



