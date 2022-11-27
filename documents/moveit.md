# Python で操作するときに使えそうな情報まとめ

## [moveit_commander](https://robo-marc.github.io/moveit_documents/moveit_commander.html)

### import
C++ の MoveGroupInterface を呼び出す。
```
import moveit_commander
```

### class moveit_commander.MoveGroupCommander(name, robot_description='robot_description', ns='')
### 公式説明
指定されたグループに対してコマンドを実行します。

### 自分の言葉で補足説明
ロボット全体のリンクではなく、どこかに任意に指定されたリンクに対して操作を実行することができるようになるクラス

例えば、一つのロボットでも「右腕」と「左腕」の二つのアームがあった時に、右腕と左腕別々でインスタンス化して別々で操作することが可能になる

そのコードが以下の通り

```
rarm = moveit_commander.MoveGroupCommander("right_arm")
larm = moveit_commander.MoveGroupCommander("left_arm")
```

#### 引数の説明
第一引数 `name` は、`srdf` 内で定義されている (`urdf` で定義されていることもあるかも？)

例えば、`CRANE-X7` では、`arm` という名前のグループと、`gripper` という名前のグループがある

これらは、`crane_x7_ros/crane_x7_moveit_config/config/crane_x7.srdf` 内で定義されている。

#### 参考
[MoveIt! の Python API を使って NEXTAGE を動かしてみよう](http://daikimaekawa.github.io/ros/2014/06/08/ROSNextage02)

#### __init__(name, robot_description='robot_description', ns='')
このインターフェースを利用する対象となるグループ名を指定します。初期化に失敗した場合、例外をスローします。

#### get_current_joint_values()
現在の関節角度をリストとして取得します（ROSの/joint_states トピックで公開されているのと同じ値です）。

#### go(joints=None, wait=True)
目標を設定し、グループを指定された目標に移動します。

#### set_joint_value_target(arg1, arg2=None, arg3=None)
関節角目標を指定します。 第一引数の型がdict、list、JointStateメッセージのいずれかである場合、他の引数は設定できません。
dict型の場合、関節名とその目標角度のペアを指定する必要があります。
list型の場合、グループのすべての関節の角度値を指定する必要があります。 
JointStateメッセージの場合は、関節角度を選択的に指定できます。 
第一引数の型が文字列の場合、第二引数には、関節角度または関節角度のリストのいずれかを設定します。
これは、特定の関節を特定の角度に設定する命令として解釈されます。
第一引数の型がPoseまたはPoseStampedの場合、第二引数と第三引数の両方を定義できます。
第二引数または第三引数が定義されている場合、それらの型は文字列またはbool型でなければなりません。
文字列型の引数は、Pose目標が指定されたエンドエフェクタの名前です（指定されない場合、デフォルトのエンドエフェクタが使用されます）。
第三引数のboolは、指定されたPoseが近似であるかどうかを決定するために使用されます（デフォルトはfalse）。
Pose目標を用いると、逆運動学を用いるて関節目標を設定できます。
ここでは、プランナーを用いて逆運動学の計算を複数試行するわけではありません。
Poseから計算されたある1つの逆運動学の解がプランナーに送信されます。

#### set_pose_target(pose, end_effector_link='')
エンドエフェクタの位置姿勢を設定します。
Poseメッセージ、PoseStampedメッセージ、または6次元のリスト形式で指定してください。




