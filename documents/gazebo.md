# ROS + Gazebo + Python Technique
# Messages
## gazebo/ModeStates
### Definition
``` Python
string[] name
geometry_msgs/Pose[] pose
geometry_msgs/Twist[] twist
```

|属性|説明
|:---:|---
|`name`|オブジェクトの名前が入る
|`pose`|オブジェクトの現在の位置姿勢が入る<br>詳細は別記事 `geometry_msgs/Pose` の項目
|`twist`|オブジェクトの移動量を入れる<br>詳細は別記事 `geometry_msgs/Twist` の項目

要は Gazebo 上の全てのオブジェクトの、名前と位置姿勢と移動量を把握するためのもの。

