# 
## image_listener_test.py
### 概要
Gazebo 上にある RealSense D435 の画像を Subscribe して MediaPipe を通して顔の Landmark をプロットした画像を保存するプログラム

### このスクリプトファイルを作成するまでの経緯

Gazebo 上にある RealSense D435 の画像を Subscribe できるようになりたかった

Subscribe できたとして、顔のイメージから Mediapipe を通して口の座標検出ができるかどうかを知りたかった

これらを実現するのに、以下の処理をするようなプログラムを組んだ

1. トピック `/camera/fake_cam_color/image_raw' に Publish されている `sensor_msgs.msg/Image` 型のメッセージを Subscribe する
2. `sensor_msgs.msg/Image` 型のイメージを `numpy.ndarray` 型イメージに変換する
3. 画像を保存する
4. 300 枚保存したら 0 枚目から上書きしていく

## mouth_print.py
### 概要
Gazebo 上にある RealSense D435 の画像から口の中心座標を取得する

### このスクリプトファイルを作成するまでの経緯

landmark を取得することに成功した

しかし、ここから口の中心座標を計算して Publish する必要がある

最初は Publish する代わりに標準出力して動作を確認する
