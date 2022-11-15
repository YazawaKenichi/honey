# ROS related commands

## Master
### ROS Master の起動
ros をバックグラウンドで立ち上げる
```
roscore &
```

## Package
### パッケージの実行
```
roslaunch PACKAGENAME LAUNCHNAME.launch
```

### パッケージのディレクトリに移動
```
roscd PACKAGENAME
```

## Topic
### 現在立ち上がっているトピックの一覧を表示
全て表示
```
rostopic list
```
検索表示
```
rostopic list | grep TOPICNAME
```

### 現在立ち上がっているトピックに関係する情報表示
トピックの `Type`, `Publishers`, `Subscribers` が表示される
```
rostopic info TOPICNAME
```

### Topic Publisher Subscriber の関係図を表示
```
rosgraph
```
