# ROS 操作のまとめ
## ROS の起動
```
$ roscore &
$ roslaunch crane_x7_gazebo crane_x7_with_table.launch
$ rosrun <PACKAGE> <SCRIPT>
```

## 新しいパッケージの作成
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg <NEWPACKAGE> <INCLUDEPACKAGE>
$ cd <NEWPACKAGE>
$ mkdir scripts config models worlds launch
$ cd scripts
$ vim <PYTHON_EXECUTIVE_FILE>
$ (cd ~/catkin_ws && catkin_make)
```

### example
```
$ catkin_create_pkg hand_test rospy
```

# モデルの作成
## CAD ソフトにてモデルを STL で出力する
ファイルの保存先は任意だが、``` models/meshes ``` が推奨
## Gazebo
### Gazebo Model Editor を開く
1. Gazebo の起動
1. ```[Ctrl] + [M]``` で Model Editor を開く
1. 左側のパレットで ```Model``` タブを選択し、```Model Plugins``` の ```Add``` から ```STL``` ファイルを選択する
1. 追加されたモデルをダブルクリックすると、モデルの位置や回転、慣性モーメント、見た目などを設定することができる

Model Editor ではモデルを直感的に作成することが可能。

具体的には以下のようなことができる
- 単純な形状の作成
- メッシュの追加
- ジョイントの追加
- モデルの編集
- リンクの編集
- ジョイントの編集
- モデルの保存

必要な設定をして、モデルを作成すると、モデルのディレクトリ・SDF・構成ファイルなどを作成してくれる。

# エラーと対処法まとめ
## Gazebo プラグイン設定が見つからない
### 状況
新しくパッケージを作成し、Python スクリプトを記述したので
```(cd ~/catkin_ws && catkin_make)``` を実行しようとした結果
以下のようなエラーが表示された。

その前に、Python のリンクを Python2 から Python3 に変更することをしていた。
### エラー文
```
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
 Could not find a package configuration file provided by "gazebo_plugins" with any of the following names:

    gazebo_pluginsConfig.cmake
    gazebo_plugins-config.cmake

 Add the installation prefix of "gazebo_plugins" to CMAKE_PREFIX_PATH or set "gazebo_plugins_DIR" to a directory containing one of the above files. If "gazebo_plugins" provides a separate development package or SDK, be sure it has been installed.
Call Stack (most recent call first):
 crane_x7_ros/crane_x7_bazebo/CMakeLists.txt:4 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/shiokaze/catkin_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/shiokaze/catkin_ws/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
```
### 原因
不明
### 対処法
以下を実行したら、上記のエラーは回避された。
```
$ cd <GITPATH>
$ cd ros_setup_scripts_Ubuntu20.04_desktop
$ ./step1.bash
```
## ros/ros.h が見当たらない
### 状況
「Gazebo プラグインが見つからない」問題を対処するために ```./setup1.bash``` を実行した後、``` (cd ~/catkin_ws && catkin_make) ``` を実行した。
### エラー文
```
/home/shiokaze/catkin_ws/src/hand_test/src/my_hello.cpp:1:10: fatal error: ros/ros.h: そのようなファイルやディレクトリはありません
    1 | #include <ros/ros.h>
      |          ^~~~~~~~~~~
compilation terminated.
make[2]: *** [hand/test/CMakeFiles/my_hello.dir/build.make:63: hand_test\CMakeFiles/my_hello.dir/src/my_hello.cpp.o] エラー 1
make[1]: *** [CMakeFiles/Makefile2:1287: hand_test\CMakeFiles/my_hello.dir/all] エラー 2
make[1]: *** 未完了のジョブを待っています....
[ 96%] Generating Python msg __init__.py for robotdesign3_2021_1
[ 96%] Built target robotdesign3_2021_1_generate_messages_py
make: *** [Makefile:141: all] エラー 2
Invoking "make -j4 -l4" failed
```
### 原因
ros/ros.h が見つからないことが原因であるが、どうして見つからないのかがわからない。

ros/ros.h が本来どこにあるべきなのかもわからないが、とりあえず ``` /usr/include ``` 内にはなかった。
### 対処法
そもそもとして、```ros/ros.h``` を必要としているらしい ``` my_hello.cpp ``` は、なくてもいいので削除した。

それに伴って、```hand_test/CMakeFiles``` に記述されていたリンクも削除しておいた。

## 有効な ROS ベース名ではない
### 状況
新しくパッケージを作成したので、以前作成したパッケージからソースコードを丸パクリして ```rosrun``` した
### エラー文
```
/home/shiokaze/catkin_ws/src/honey/scripts/hoge.py:92: UserWarning: 'packman!' is not a legal ROS base name. This may cause problems with other ROS tools.
  rospy.init_node("packman!")
```
### 原因
```rospy.init_node("packman!")``` がすでにほかのスクリプトから使用されていたから？
### 対処法
``` "hoge" ``` に変更したらなくなった

## rospy.iniit_node() は既に異なる引数で呼び出されています
### 状況
新しくパッケージを作成したので、以前作成したパッケージからソースコードを丸パクリして ```rosrun``` した
### エラー文
```
Traceback (most recent call last):
  File "/home/shiokaze/catkin_ws/src/honey/scripts/hoge.py", line 95, in <module>
    main()
  File "/home/shiokaze/catkin_ws/src/honey/scripts/hoge.py", line 71, in main
    rospy.init_node("gipper_action_client")
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/client.py", line 274, in init_node
    raise rospy.exceptions.ROSException("rospy.init_node() has already been called with different arguments: "+str(_init_node_args))
rospy.exceptions.ROSException: rospy.init_node() has already been called with different arguments: ('hoge', ['/home/shiokaze/catkin_ws/src/honey/scripts/hoge.py'], False, None, False, False)
```
### 原因
```rospy.init_node()``` が既に呼び出されていたことが原因
### 対処法
実際にコードを見たら複数あったので、片方消した

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

## 
### 状況
### エラー文
### 原因
### 対処法

# ROS 環境の構築方法
```
$ sudo apt-get update
$ sudo apt-get upgrade

$ git clone https://github.com/ryuichiueda/ros_setup_scripts_Ubuntu20.04_desktop
$ cd ros_setup_scripts_Ubuntu20.04_desktop
$ sudo chmod +x step0.bash step1.bash
$ ./step0.bash
$ ./step1.bash
$ source ~/.bashrc

$ roscore ROS Master, ROS Parameter Server, rosout ログ用の node の立ち上げ。

$ mkdir -p catkin_ws/src -p で階層構造になったディレクトリを作成する。

$ catkin_init_workspace ワークスペースが作成できるコマンド。

$ catkin_make catkin パッケージをビルドすることができる。 実行するときは必ず catkin_ws 内で catkin_make する。

$ ( cd catkin_ws && catkin_make) いちいち移動が面倒くさい場合は上記のようなコマンドで実行できる。

$ vim ~/.bashrc で以下の行を追加。

source ~/catkin_ws/devel/setup.bash
$ source ~/.bashrc

必要な追加パッケージをクローンしてくる。 $ git clone https://github.com/rt-net/crane_x7_ros.git $ git clone https://github.com/rt-net/crane_x7_description.git $ git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git

$ rosdep update

$ ( cd catkin_ws && catkin_make )

bashrc の更新 $ source ~/.bashrc

~/.ignition/fuel/config.yaml の作成。 $ mkdir -p ~/.ignition/fuel/ $ cd ~/.ignition/fuel $ vim config.yaml

config.yaml に以下を記述

servers:
-
  name: osrf
  url: https://api.ignitionrobotics.org
$ rosdep install -r -y --from-paths --ignore-src crane_x7_ros

$ ( cd catkin_ws && catkin_make )

bashrc の更新。 $ source ~/.bashrc

roscore をバックグラウンド起動。 $ roscore &

rviz の起動。 $ rviz

CRANE-X7 のサンプルを立ち上げる。 $ roslaunch crane_x7_gazebo crane_x7_with_table.launch

上のコマンドが実行できている状態で $ rosrun crane_x7_examples gripper_action_example.py とすることで、gripper_action_example.py のサンプルコードを実行してくれる。 このサンプルコードを改造していけば好き放題アームを動かせるようになるっぽい。

サンプルプログラムは Python で記述されている。 アームの動きを制御するために、サンプルプログラムは他のプロセスに情報（例えば「アームを開く角度」など）を送る。 ここで ROS Topic を見てみる。 goal トピックはサンプルプログラムの「アームを開く角度情報」をキャッチするためのライン。 $ rostopic list | grep goal で表示されるもののどれかがそれ。 result トピックは処理結果。 $ rostopic list | grep result

ROS プログラミングの内容としては、この「ロボットを直接制御する」ではなく、「C++ や Python など書きやすい言語を用いて、他のプロセスに制御情報を与える。」というのが ROS プログラミング。

自身の ROS パッケージを作成する。 $ cd catkin_ws/src $ catkin_create_pkg hand_test rospy catkin_create_pkg で ROS パッケージを作成する。 パッケージ名に hand_test を指定する。 rospy というモジュール・パッケージを使う。

Python でなんかしらコードを記述する。

catkin_make する。 bashrc を更新する。 roslaunch crane_x7_gazebo crane_x7_with_table.launch して、サンプルコードの出すメッセージの受け取り手（サブスクライバー先にいる）ノードを起動する。 rosrun hand_test hogehoge.py して、hand_test パッケージの hogehoge.py というコードを実行。

Gazebo 構成要素 World ファイル 静的オブジェクトの記述。 Model ファイル モデルデータの記述。 World ファイルに以下の記述をすれば、Model フィアルを World ファイルに含めることが可能。 model://model_file_name

環境変数 GAZEBO_MODEL_PATH : モデルを検索するディレクトリ GAZEBO_RESOURCE_PATH : world ファイルや media ファイルのリソースを検索するディレクトリ GAZEBO_MODEL_DATABASE_URI : モデルをダウンロードするオンラインモデルのデータベースの URI GAZEBO_MASTER_URI : Gazebo マスタの URI GAZEBO_PLUGIN_PATH : Gazebo 実行時にプラグイン共有ライブラリを検索するディレクトリ

新しいモデルを追加する。 iris_2d_lidar という名前でモデルを追加する。

$ cd ~/catkin_ws/src/projectname
$ mkdir -p models/iris_2d_lidar
SDF ファイルを作成する

まとめ roscore で ROS の受け取り口の起動。 roslaunch crane_x7_gazebo crane_x7_with_table.launch で gazebo に crane を起動。 rosrun crane_x7_examples gripper_action_example.py でサンプルコードの実行をする。

roslaunch は受け取りノードの実行。（ Subscliber の起動？） rosrun は送信ノードの実行。（ Publisher の起動？）
```
