# フォークモデルを Gazebo 上に出現させる
## フォークモデルを作成する
1. Autodesk Inventor で作成した
1. STL ファイルで出力する

    ** この時、オプションから STL の単位を メートル にすることを忘れないこと！ **
## Gazebo 上で表示する
1. Gazebo を開く
1. Gazebo Model Editor を開く
1. Custom Shapes から保存した STL ファイルを指定する
## インポートしたモデルの情報を入力する
1. モデルをダブルクリックして Link Inspector を開く
1. ここで Pose や Inertial を指定する

    ちなみに `2 / PI` はおよそ `1.570796`
## Gazebo Model Editor を閉じる
1. 保存先を聞かれるので、プロジェクトの models を指定して名前を fork にする
1. 保存すると Gazebo の Insert に fork ができているはず...

## 参考サイト

[Tutorial : Gazebo Model Editor (Englis)](https://classic.gazebosim.org/tutorials?tut=guided_b3)


