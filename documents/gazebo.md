# フォークモデルを Gazebo 上に出現させる
## フォークモデルを作成する
1. Autodesk Inventor で作成した
1. STL ファイルで出力する

    ** この時、オプションから STL の単位を メートル にすることを忘れないこと！ **

1. STL ファイルは `model/meshes` 推奨（別にここじゃなくてもいい）
1. Gazebo を単体で開く
1. `[Ctrl] + [M]` で Gazebo Model Editor を開く
1. 左側のパレットで ```Model``` タブを選択し、```Model Plugins``` の ```Add``` から ```STL``` ファイルを選択する
1. 追加されたモデルをダブルクリックすると、モデルの位置や回転、慣性モーメント、見た目などを設定することができる

    ちなみに `2 / PI` はおよそ `1.570796` なので、必要に応じて回転させる

1. 保存先を聞かれるので、プロジェクトの models を指定して名前を fork にする
1. 保存すると Gazebo の Insert に fork ができているはず...

## フォークと机とクレーンと
1. crane_x7_with_table.launch を Gazebo で実行する
1. 左側の Insert から fork を選択肢、一旦テキトーなところをクリックする
1. fork のポジション `pose` を

|pose|value
|---|---
|x|0.3
|y|0
|y|1.02
|roll|0
|pitch|0
|yaw|3.141593

のように適切な値にする
1. Gazebo のツールバーから World ファイルを保存する
    これで、あとはこの world ファイルを `$ gazebo hoge.world` とすればいつでも同じ状況が作り出せる

## 参考サイト

[Tutorial : Gazebo Model Editor (Englis)](https://classic.gazebosim.org/tutorials?tut=guided_b3)

# Gazebo を使う上で知っておきたい知識？
## SDF ファイルの文法は XML
## Model ファイルと World ファイルの違い
> Model -> モデルを定義する
> World -> いくつかのモデルで構成されるワールドを定義する
## URDF ファイルと SDF ファイルの違い
> URDF -> ROS におけるモデルの定義の形式
> SDF -> Gazebo におけるモデルの定義の形式
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

