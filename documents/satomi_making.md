# CRANE-X7 に RealSense を搭載する
1. パッケージのルートディレクトリに launch ディレクトリを作成
    ```
    $ mkdir launch
    ```

1. satomi.launch ファイルを作成

    ゼロから launch ファイルを作成するのは面倒くさい（文法の勉強とかしてる暇ない）ので、すでに作成されている launch ファイルからコピペして新しい launch ファイルを作る。

    1. 必要なリポジトリを clone
        RealSense の launch ファイルと urdf ファイルを、[Kuwamai](https://github.com/Kuwamai/crane_x7_d435)様の GitHub から clone させてもらいます。
        ```
        $ cd ~/catkin_ws
        $ git clone https://github.com/Kuwamai/crane_x7_d435
        $ (cd ~/catkin_ws && catkin_make)
        $ source ~/catkin_ws/devel/setup.sh
        ```

    1. 必要なファイルを自分のパッケージ内にコピー
        `./catkin_ws/src`
        ```
        $ cp crane_x7_d435/launch/bringup_sim.launch honey/launch/bringup_sim.launch
        ```

    1. `crane_x7_gazebo` の `crane_x7_with_table.launch` と、`crane_x7_d435` の `bringup_sim.launch` を合体させる

        `bringup_sim.launch`
        ```
        <include file="$(find crane_x7_gazebo)/launch/crane_x7_with_table.launch" />
        ```
        上記の部分を、下記の部分に置き換える。

        `crane_x7_with_table.launch`
        ```
        <arg name="use_gui" default="false" />
        ... 略 ...
            <arg name="use_gui" default="$(arg use_gui)" />$
        </include>
        ```
        つまり、わざわざ `bringup_sim.launch` で `include` して読み込んでいるものを、直接ファイルに書き込んでやろうという話。

    1. 書き換えた `bringup_sim.launch` を `satomi.launch` にする
        ```
        $ mv bringup_sim.launch satomi.launc
        ```

1. launch ファイルを実行する
    ```
    $ roslaunch honey satomi.launch
    ```    
    すると `CRANE-X7` のグリッパーの真横あたりに四角い箱が追加されていることがわかる。
    
    これが `RealSense D435 となる。

### さっさとカメラを起動したい
あるいは
### とりあえずここまでで、正しく動作できるかを確認したい場合は以下を実行する
1. Gazebo と RViz が開いたら、RViz で `Displays` の `Add`
1. `Create visualization` の `By topic` タブを開く
1. そのタブから `/camera/fake_cam_color/image_raw/Camera` を選択し、[OK] する
1. すると、Gazebo 空間の RealSense D435 が取得した画像を見ることができる。
# フォークモデルを Gazebo 上に出現させる
## フォークモデルを作成する
Inventor で作成したモデルを Gazebo に表示させる方法について記述する。

今回使用するフォークは、動力・リンクのないオブジェクトであるため、モデルを Gazebo 上に表示させて使用する最も簡単な方法となっている。

また、実際に使用したフォークモデルの STL ファイルは [こちら](https://github.com/yazawakenichi/honey/tree/main/models/meshes/fork) にあり、
Autodesk Inventor Pro 2022 で作成したパーツ・アセンブリファイルは、[こちら（GrabCAD）](https://https://workbench.grabcad.com/workbench/projects/gcLbEpjExKeGrawJHAMI79yPYM8IIow4o7FhAo-OEd0hD7#/folder/12981129)に用意した。

0. Inventor でフォークモデルを作成する
    今後しょうもない部分で苦戦しないようにするために、以下に意識してモデルを作成する。
    - 原点はグリッパーが掴む場所の中心に置く
        自分で慣性モーメントを計算する必要がなくなる
        パーツファイルを作成しているうちは考えなくて良いが、アセンブリファイルを作ってパーツファイルを組み立てたら、絶対座標系の xyz 平面とアセンブリを拘束することで、アセンブリを原点に固定する。
    - グリッパーが開閉する方向は、RealSense の水平方向である
        ものを掴んだときにカメラが遮られる可能性がある
    - 出力する STL データは単位をメートルにする

1. Autodesk Inventor で作成したフォークモデルを STL ファイルで出力する

    ** この時、オプションから STL の単位を メートル にすることを忘れないこと！ **

1. STL ファイルは `model/meshes` に保存することを推奨（ほんとはどこでもいい）
1. Gazebo を単体で開く
1. `[Ctrl] + [M]` で Gazebo Model Editor を開く
1. 左側のパレットで ```Model``` タブを選択し、```Model Plugins``` の ```Add``` から ```STL``` ファイルを選択する
1. 追加されたモデルをダブルクリックすると、モデルの位置や回転、慣性モーメント、見た目などを設定することができる

    ちなみに `PI / 2` はおよそ `1.570796` なので、必要に応じて回転させる

1. 保存先を聞かれるので、プロジェクトの models を指定して名前を fork にする
1. 保存すると Gazebo の Insert に fork ができているはず...

## フォークと机とクレーンと
Gazebo 上でシミュレートする環境を簡単に起動できるようにしたい。

そのために状況を記述した world ファイルと、それを読み込むための launch ファイルを作成する。

### 1.world ファイルを作成する
1. satomi.launch を Gazebo で実行する
1. 左側の Insert から fork を選択肢、一旦テキトーなところをクリックする
1. fork のポジション `pose` を

    |pose|value
    |---|---
    |x|0.2
    |y|0.5
    |z|1.2
    |roll|1.570796
    |pitch|0
    |yaw|3.141593

    のように適切な値にする
1. Gazebo のツールバーから World ファイルを保存する
    ここでは保存した world ファイル名を `satomi.world` とする

### 2.launch ファイルから world ファイルを読み込む
roslaunch は ROS ノードを起動し、ROS でロボットを起動するための方法の一つ。

この方法で、先程保存した world ファイルを開く

1. satomi.launch の書き換え
    ```
    <arg name="world_name" value="$(find crane_x7_gazebo)/worlds/table.world"/>
    ```
    の部分の、crane_x7_gazebo を 自分のパッケージ名にし、```table.world```
1. 実行
    ```
    $ roslaunch honey satomi.launch
    ```

## Node を作成して立ち上げる
ここでは `sample.py` を作成済みであると仮定する。

### 実行

```
rosrun honey sample.py
```

を実行するだけで `sample.py` が起動する。

```
./sample.py
python sample.py
```

などでも使用可能

毎回いちいち実行するのは面倒なので `launch` ファイルに記述して自動でノードを起動するようにする

### 自動実行するようにする

`satomi.launch` に以下の行を追加する。

```
<node name="sample.py" pkg="honey" type="sample.py" />
```

これだけで `satomi.launch` を起動した瞬間 `sample.py` を実行してくれるようになる。

#### 躓いたこと
`---.py: error: unrecognized arguments: __name:=--- __log:=/home/---/.ros/log/--------------/---------.log`
のような文章が大量に高速で表示されるときの対処法（rospy.md にも記述済み）

`sample.py` を以下のように追記する。

```
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--text', '-t', type=str, dest='text', help='set message to speech')
    parser.add_argument('--label', '-l', metavar='L', type=str, nargs='?', dest='label', help='set label for this message')
    parser.add_argument('-p', action='store_true', dest='periodic_flag', help='set either periodic message or not')
    args = parser.parse_args()

    rospy.init_node( ...
```

ROS launch を実行したときに python スクリプトに引数が渡されているかのような文章が見受けられるので perser を用いて引数を処理する

##### 参考
[[easy] launch file syntax: how to pass quotation mark to a python script](https://answers.ros.org/question/338617/easy-launch-file-syntax-how-to-pass-quotation-mark-to-a-python-script/)

## 参考サイト
- [Tutorial : Gazebo Model Editor (Englis)](https://classic.gazebosim.org/tutorials?tut=guided_b3)
- [ROS講座37 gazebo worldを作成する (Japanese)](https://qiita.com/srs/items/9b23ad12bea9e3ec0480)
- [ROS入門 (20) - ROS1のlaunchファイルの利用 (Japanese)](https://note.com/npaka/n/na4d2beadf995)
- [GitHub - Kuwamai / crane_x7_d435 (Japanese)](https://github.com/Kuwamai/crane_x7_d435)

# Gazebo を使う上で知っておきたい知識？
## SDF ファイルの文法は XML
## Model ファイルと World ファイルの違い
> Model -> モデルを定義する
> World -> いくつかのモデルで構成されるワールドを定義する
## URDF ファイルと SDF ファイルの違い
> URDF -> ROS におけるモデルの定義の形式
>  SDF -> Gazebo におけるモデルの定義の形式
というだけ

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

## SDF ファイルと SRDF ファイルの違い
>  SDF -> Gazebo のシミュレーション記述フォーマット
> SRDF -> MoveIt! のロボットセマンティック記述ファイル

SDF は Gazebo で使うもの

SRDF は MoveIt! で使うもの

-> 全然違う！


