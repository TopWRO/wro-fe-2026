# 纯拍照：对照 Recording_N02.py 修正版

在树莓派上替换 camera_photo.py，使用能运行原录制程序的同一个 Python 环境。退出旧相机程序后，先不带亮度、对比度参数运行：

```bash
python3 camera_photo.py
```

默认采集 640×480。点击预览窗口，按 S 或 s 保存一张 PNG 到程序旁的 photos 文件夹。按 Q / Esc 或 Ctrl+C 退出。

需要对照原录制程序的采集分辨率时：

```bash
python3 camera_photo.py --width 320 --height 240
```

原录制程序采集 320×240 后缩放到 160×120 写入 Tub；本版按采集分辨率直接保存，不做这次缩放。

## 相机设置对照

本版严格沿用原程序的 PiCamera 入口、等待 2 秒、以下属性设置顺序、后台相机线程及 20 Hz 主循环：

```python
camera.framerate = 30
camera.exposure_mode = 'off'
camera.awb_mode = 'off'
camera.iso = 100
camera.shutter_speed = 15000
camera.exposure_compensation = 0
camera.awb_gains = (1.2, 1.5)
```

原录制程序没有显式设置亮度、对比度，因此本版默认也不额外设置。

上一版加入了 Picamera2 的 set_controls() 曝光、增益和白平衡控制。若你们实际使用 Picamera2，原程序的旧版属性可能并未控制传感器，上一版新增的控制却会真正关闭自动曝光/白平衡，并设置增益 1.0。这是偏暗、偏色的可能原因，需要实机验证。

本版优先复现原代码的调用行为，不自动把旧属性翻译为新曝光/白平衡控制。这不表示旧属性在所有后端都有效，而是为了与已验证的录制程序一致对照。

预览和保存采用与原录制程序相同的 RGB 转 BGR：

```python
bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
```

不要保留之前临时测试的 bgr = rgb.copy()。

## 可选参数

先确认默认照片颜色，再测试：

```bash
python3 camera_photo.py --brightness 60 --contrast 10
```

亮度范围 0～100，中性值 50；对比度范围 -100～100，中性值 0。不传参数就不额外设置。旧后端使用属性，新后端仅对明确传入的这两个参数调用对应控制，不修改曝光或白平衡。

自定义保存目录：--output-dir ~/camera_photos。

启动时会打印相机后端类型和 DonkeyCar 摄像头模块路径，方便确认环境。已检查默认属性值、默认不调用新控制、可选控制隔离和照片颜色顺序；尚未连接真实树莓派验证。
