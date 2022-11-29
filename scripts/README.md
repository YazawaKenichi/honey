# 
## mouth_pub.py
Gazebo 上の RealSense D435 のイメージから口の中心座標を計算し、トピック `/MouthCoordinate` に Publish するスクリプト

### Topic : MouthCoordinate
|Topic
|:---:
|Name|MouthCoordinate
|Type|Int16MultiArray

### 大まかな処理内容
1. Topic `/camera/fake_cam_color/image_raw` に Publish されている `sensor_msgs.msg/Image` 型のメッセージを Subscribe する
2. Subscribe されたら、`sensor_msgs.msg/Image` 型のメッセージを `numpy.ndarray` 型に変換する
3. MediaPipe に渡し、口の特徴点 landmark を取得する
4. 口の landmark の中心座標（平均座標）を計算する
5. 口の中心座標を `numpy.ndarray` 型から `std_msgs.msg/Int16MultiArray` 型に変換
6. 変換された座標を、トピック `MouthCoordinate` に Publish する

