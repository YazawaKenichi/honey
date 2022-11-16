# NumPy Scripting Technique

## import
```
import Numpy as np
```

## 任意配列を作成する
### 直接リストの作成
```
ARRAY = np.array([[x0, y0, z0, ... ], [x1, y1, z1, ... ], [x2, y2, z2, ... ], ... ])
```

np.array() が関数、引数には配列を渡す必要がある。

だから、横の要素だけでなく、縦の要素も `[` と `]` で囲む必要がある。 

出来上がる配列は以下

| x | y | z | ...
|:---:|:---:|:---:|:---:
|x0|y0|z0|...
|x1|y1|z1|...
|x2|y2|z2|...
|...|...|...|...

### 横ベクトルを縦に重ねていく方法
```
ARRAY1 = np.array([x0, y0, z0, ... ])
ARRAY2 = np.array([x1, y1, z1, ... ])
ARRAY3 = np.array([x2, y2, z2, ... ])
    ...
ARRAY = np.array([ARRAY1, ARRAY2, ARRAY3, ... ])
```

### 横ベクトルの下に、新たに横ベクトルを追加していく方法

```
ARRAY_LINE[0] = np.array([x0, y0, z0, ... ])
ARRAY_LINE[1] = np.array([x1, y1, z1, ... ])
ARRAY_LINE[2] = np.array([x2, y2, z2, ... ])
    ...
ARRAY_LINE[N] = np.array([xN, yN, zN, ... ])

for i in range(N):
    ARRAY.append(ARRAY_LINE[N])
```

### 参考
[NumPy Reference](https://numpy.org/doc/stable/reference/generated/numpy.array.html?highlight=array#numpy.array)

## 転置行列の作成
```
ARRAY_T = ARRAY.T
```
