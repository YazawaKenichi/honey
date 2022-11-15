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
コールバック関数の中で呼び出す
```
rospy.spin()
```
