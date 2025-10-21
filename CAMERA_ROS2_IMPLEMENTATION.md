# Lunalab TopView Camera ROS2 パブリッシャー実装

このドキュメントでは、`/Lunalab/Cameras/TopView` Prim pathのカメラ画像をROSトピックとして流す実装について説明します。

## 実装内容

### 1. **lunalab_ros2.py** の修正

#### インポートの追加
```python
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
```

#### 初期化（`__init__`）での追加項目
```python
# カメラ関連の属性を初期化
self.bridge = CvBridge()
self.topview_camera_path = "/Lunalab/Cameras/TopView"
self.image_publisher = self.create_publisher(Image, "/OmniLRS/Lunalab/TopView/image_raw", 1)
self.camera_info_publisher = self.create_publisher(CameraInfo, "/OmniLRS/Lunalab/TopView/camera_info", 1)
self.camera_frame_id = "lunalab_topview_camera"
self.sequence_counter = 0
```

#### 定期更新メソッド（`periodic_update`）の実装
```python
def periodic_update(self, dt: float) -> None:
    """
    Periodic update to publish camera images.
    """
    self.publish_topview_camera_image()
```

#### 追加メソッド
- `get_camera_image_from_prim()`: カメラPrimからRGB画像を取得
- `publish_topview_camera_image()`: 画像をROSトピックにパブリッシュ
- `publish_camera_info()`: カメラ内部パラメータ情報をパブリッシュ

### 2. **camera_helper.py** の新規作成

カメラ画像取得のユーティリティ関数を提供します：

- `get_rgb_image_from_viewport()`: Omniverse Replicator APIを使用してRGB画像をキャプチャ
- `get_depth_image_from_viewport()`: 深度画像をキャプチャ
- `get_camera_matrices()`: カメラの内部・外部パラメータを取得

## ROS2 トピック

### パブリッシャー

| トピック | メッセージ型 | 説明 |
|---------|-----------|------|
| `/OmniLRS/Lunalab/TopView/image_raw` | `sensor_msgs/Image` | TopViewカメラからのRGB画像 |
| `/OmniLRS/Lunalab/TopView/camera_info` | `sensor_msgs/CameraInfo` | カメラキャリブレーション情報 |

### 画像仕様

- **解像度**: 512 x 512 ピクセル
- **エンコーディング**: RGB8
- **フレームID**: `lunalab_topview_camera`

## 使用方法

### 1. 依存関係の確認

以下のパッケージが必要です：

```bash
# ROS2パッケージ
ros-humble-sensor-msgs
ros-humble-cv-bridge

# Python パッケージ
opencv-python
numpy
```

Dockerfileで既にインストール済みであることを確認:
```dockerfile
RUN apt-get install -y ros-humble-cv-bridge ros-humble-image-transport
```

### 2. ノードの起動

```bash
# standard ROS2 launch
ros2 run omnilrs lunalab_ros2
```

### 3. 画像の確認

```bash
# image_viewで画像を表示
ros2 run image_view image_view --ros-args -r image:=/OmniLRS/Lunalab/TopView/image_raw

# rqt_imageで確認
rqt_image_view /OmniLRS/Lunalab/TopView/image_raw

# rosbag で記録
ros2 bag record /OmniLRS/Lunalab/TopView/image_raw
```

## カスタマイズ

### 画像解像度の変更

`lunalab_ros2.py` の以下の行を変更:

```python
def get_camera_image_from_prim(self):
    image_data = get_rgb_image_from_viewport(
        prim_path=self.topview_camera_path,
        width=1024,      # ← ここを変更（デフォルト: 512）
        height=1024,     # ← ここを変更（デフォルト: 512）
    )
```

### パブリッシュレート の変更

`periodic_update` の呼び出し頻度はシミュレーションの更新レートに依存します。
呼び出し頻度を制御するには、タイムステップを使用:

```python
def periodic_update(self, dt: float) -> None:
    # dt = シミュレーション時間ステップ
    # 60 Hzの場合: dt ≈ 0.0167秒
    self.publish_topview_camera_image()
```

### カメラパラメータの調整

`publish_camera_info()` メソッド内の以下を編集:

```python
# カメラ内部パラメータ (焦点距離, 主点)
fx = 512.0      # ← 焦点距離 X
fy = 512.0      # ← 焦点距離 Y
cx = 256.0      # ← 主点 X
cy = 256.0      # ← 主点 Y
```

## トラブルシューティング

### 画像が真っ黒/真っ白の場合

1. TopViewカメラが正しく配置されているか確認
   ```bash
   usdcat assets/USD_Assets/environments/Lunalab.usd | grep -A 5 "TopView"
   ```

2. カメラの可視性を確認
   ```python
   from pxr import Usd, UsdGeom
   stage = Usd.Stage.GetCurrentStage()
   camera = stage.GetPrimAtPath("/Lunalab/Cameras/TopView")
   print(camera.GetTypeName())  # "Camera"であることを確認
   ```

### インポートエラーが出る場合

- Omniverseが正しく起動しているか確認
- `cv_bridge` が install されているか確認:
  ```bash
  pip list | grep cv-bridge
  ```

### パブリッシャーが何も出力しない場合

1. ROS2 ノードが起動しているか確認:
   ```bash
   ros2 node list
   ```

2. トピックをリスン:
   ```bash
   ros2 topic list
   ros2 topic echo /OmniLRS/Lunalab/TopView/image_raw
   ```

## 参考資料

- [ROS2 Image Transport](http://wiki.ros.org/image_transport)
- [Omniverse Replicator](https://docs.omniverse.nvidia.com/py/replicator/)
- [OpenCV cv_bridge](http://wiki.ros.org/cv_bridge)
