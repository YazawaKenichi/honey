# Honey

## パッケージについて
このパッケージは[株式会社アールティ様のパッケージ]を使用して、千葉工業大学未来ロボティクス学科 2022 年度 設計製作論 3 知能コース において、チーム「転ばぬ先のソイヤ」が作成したものです。

## 概要

このリポジトリは、株式会社アールティ様が販売されている CRANE-X7 マニピュレータを制御し、あーんしてもらうパッケージです。

まだ製作途中であります。

## 動作環境
OS : Ubuntu 20.04 LTS
ROS : Noetic
Gazebo : 11.11.0
Rviz : 1.14.19

## 使用する道具
CRANE-X7
RealSense D435
持ち手を持ちやすく改造したフォーク

## セットアップ方法

1. このパッケージの clone
```
cd ~/catkin_ws/src
git clone https://github.com/yazawakenichi/honey
```

2. 必要なパッケージの clone
```
git clone https://github.com/rt-net/crane_x7_ros
git clone https://github.com/Kuwamai/crane_x7_d435
```

3. ビルド
```
(cd ~/catkin_ws && catkin_make)
source ~/catkin_ws/devel/setup.bash
```

4. Python ライブラリのインストール
```
pip install numpy==1.19.3
pip install mediapipe
```

これで環境は整いました。

## 使用方法

1. ROS Master の立ち上げ

```
roscore &
```

### シミュレーションする場合
2. satomi_sim.world の起動
```
roslaunch honey satomi_sim.launch
```

3. スクリプトの実行

以下の方法でスクリプトを確認してください

```
cd ~/catkin_ws/src/honey/scripts
ls
```

結果

```
image_listener_test.py  satomi_sim.py
```

4. 実行するスクリプトを決めたら、以下でスクリプトを実行してください

```
rosrun honey satomi_sim.py
```

#### satomi_sim.py

Gazebo 内にある RealSense D435 の画像データから、顔を検出し、口の中心座標を terminal に print するスクリプト。

あとは、この print して満足しているデータを topic に publish すれば、画像解析ノードの処理は完成。

#### image_listener_test.py

Gazebo 内にある RealSense D435 の画像データを取得し、連番画像として 'honey/images/' に保存するスクリプト。

sensor_msgs.msg/Image 型のメッセージを取得して OpenCV で開くことができるかどうかのテストのために作成したスクリプト。

容量を圧迫するのが嫌なので 300 枚保存したら 0 番目から上書きしていく。

連番画像から動画を作成する場合は以下を実行。

'honey/videos/' に 'output.mp4' ができる。

```
cd ~/catkin_ws/honey/videos/
source append_video.bash
```

