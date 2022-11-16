# OpenCV Scripting Technique

## import
```
import cv2
```

## 画像ファイルを取得する
```
IMAGE = cv2.imread('FILENAME')
```

## 画像ファイルを表示する
```
cv2.imshow('WINDOWTITLE', IMAGE)
```

## imread がちゃんと読み込まれたか確認する
```
img = cv2.imread( ... )
if not img is None:
    正しく読まれた時の処理
else:
    正しく読まれなかった時の処理
```
### 参考
[imread がちゃんと読まれたか確認する](https://qiita.com/musaprg/items/755d361b17ff77336474)

## 複数点間の中心の座標を取得する
```
points = np.array([[x0, y0], [x1, y1], [x2, y2], ... ])

def center(points):
    # 転置行列を作成し、１行を総加算
    mid_x = statistics.mean(points.T[0])
    mid_y = statistics.mean(points.T[1])
    return np.array([mid_x, mid_y])
```
### 参考
[行列の相加平均を算出する](https://note.nkmk.me/python-statistics-mean-median-mode-var-stdev/)
