# 红色标志形状测试（640×480）

这个独立程序从 `Recording_N02.py` 提取 DonkeyCar 摄像头调用方式，实时测量集中红色区域的轮廓、像素宽高和宽高比。第一阶段仅测量，不根据比例强行判定实际高度，也不改红绿颜色。

## 参考程序检查结果

- `Recording_N02.py` 没有导入项目目录里的自定义 `.py` 文件；开头提到的其他程序是来源说明，不是运行时依赖。
- 完整参考程序需要 OpenCV、NumPy、DonkeyCar、pygame、Dynamixel SDK、Keras/TensorFlow，以及与 DonkeyCar 相配的树莓派摄像头软件栈。即使只选择 `--recording`，它也会在启动时导入 Keras；这不是相机本身必需的。
- 参考程序的摄像头为 `from donkeycar.parts.camera import PiCamera`，创建后由 DonkeyCar 后台线程取图。输出按 **RGB** 处理，不能直接当作 OpenCV 默认的 BGR。
- 原程序先采集 320×240，再用 `center_crop()` 缩放为 160×120；这个函数实际上没有裁剪。
- 原程序的 PS4 摇杆在 `run()` 中已做死区/步进量化，电机类又做一次量化；因此 Tub 中存的是**量化后的** `user/angle` / `user/throttle`，不是注释所说的完全原始摇杆值。电机的 0.40 速度缩放不写入 Tub。
- 图像和操纵量通过 DonkeyCar 同一循环写入 Tub；摄像头是异步线程，该结构不能单凭代码证明曝光时刻与摇杆采样时刻精确同步。
- `--driving` 才需要 `.h5` 模型；`--recording` 不需要模型文件，但需要可用的 PS4、电机串口和上述软件依赖。
- 本次是代码审阅，没有连接树莓派实测，因此不能据此确认你们机器上的具体 DonkeyCar / 相机驱动版本。

## 新程序沿用哪些设置

- 保留 DonkeyCar `PiCamera` 入口和后台线程模式，摄像头请求尺寸直接改为 **640×480**；分析前不缩放到 160×120。
- 保留主循环 20 Hz、相机 30 fps、快门 15000 μs、关闭自动曝光/白平衡、白平衡增益 `(1.2, 1.5)`。
- 旧 `picamera` 后端沿用原来的 ISO 100 和属性设置。
- 新 `Picamera2` 后端用真正的 `set_controls()` 应用同类手动控制；模拟增益设为 1.0 起点，**不宣称它精确等同于旧后端 ISO 100**。不同后端/传感器仍要现场比较色彩和亮度。
- RGB 输入使用 `COLOR_RGB2HSV`；预览和 PNG 写入前转换为 BGR，避免保存出的红色变成蓝色。
- 新程序不导入原录制脚本，不创建手柄、电机、Tub 或 AI 模型对象。拍照仅在按 `S` 或使用 `--image` 时发生。

## 在树莓派上运行

将整个 `surprise_shape_test` 文件夹复制到树莓派，例如放在 `Recording_N02.py` 旁边。**先激活能正常运行原程序的同一个 Python 虚拟环境**，关闭正在占用相机的录制/驾驶程序，再进入这个文件夹：

```bash
python3 red_shape_camera.py
```

不需要 `--recording`，也不需要接 PS4 手柄。用树莓派桌面或 VNC 打开，才能看到实时窗口。纯 SSH 可以查看终端测量值：

```bash
python3 red_shape_camera.py --no-window
```

沿用你们已有环境即可，不建议为运行此测试先升级 DonkeyCar。如果报相机导入错误，检查是否激活了原来的环境。

## 画面显示什么

- 左侧为真实相机画面，右侧为测量面板；测量仍基于 640×480，面板不参与分析。
- 洋红色线：实际红色区域外轮廓。
- 绿色框：与画面水平/竖直方向对齐的包围框。
- 黄色框：允许旋转的最小包围矩形，用于减小轻微画面内倾斜对比例的影响。
- 青色点：轮廓近似的顶点。`QUAD` 表示检测到凸四边形近似，`POLYGON-6` 表示六边形近似；斜看柱子可能有五/六个轮廓顶点，不会因此直接排除。
- `W/H`：宽 ÷ 高；`H/W`：高 ÷ 宽，避免比例方向混淆。
- `Rectangle fill`：轮廓面积 ÷ 黄色框面积，观察轮廓与矩形的贴合程度；这不是识别置信度。
- `CLIPPED`：轮廓接触画面边缘，可能拍不全，此时不要使用它的比例判断模型。
- 最多显示三个独立的红色区域；**#1 和右侧详细数据始终对应当前帧面积最大的目标**。这不是跨帧跟踪，背景有更大的红色物体时会选中背景。
- `loop Hz` 是处理循环速度，不是测得的相机独立新帧帧率。

正对一个竖直面、镜头近似水平且透视影响较小时，5 cm 宽模型的参考值：

| 模型高度 | 理想 W/H | 理想 H/W |
| --- | --- | --- |
| 5 cm | 1.000 | 1.000 |
| 10 cm | 0.500 | 2.000 |
| 15 cm | 0.333 | 3.000 |

**这些是参考值，不是已校准的分类阈值。** 转动一个 5×5 cm 方柱，能同时看到两个侧面时，正交投影下其水平宽度可以从约 5 cm 增至约 7.07 cm；俯视时顶面还可能增加轮廓高度。黄色框只补偿画面内旋转，不校正三维视角、镜头畸变或遮挡。因此不要把所有偏离 0.5 的结果直接解释为特殊柱。

## 调整红色检测

初始阈值沿用原录制程序：H 为 0～10 或 170～179，S≥100，V≥70。灰度/亮度变化通过 HSV 中独立的 S、V 阈值处理，不要求固定 RGB 值。

另一个窗口显示掩膜，白色为选中的红色，黑色为背景。可以实时拖动：

- `Hue +/-`：允许红色色相偏移范围；镜头偏色时适当增大，也会更容易包含橙色/偏紫背景。
- `S min`：饱和度下限；降低可容忍发白的红色，但也可能纳入背景。
- `V min`：亮度下限；降低可包含暗红侧面，但可能带入暗噪声。
- `Min area`：过滤小块噪点，单位为轮廓面积 px²。

默认仅用 3×3 小核闭运算/开运算，填补小缺口并去孤立噪点。大面积高光变成白色、阴影变成黑色，或红色被分割成两块时，不能保证恢复真实边界；应观察掩膜并改善光线。可关闭形态学处理比较边界：

```bash
python3 red_shape_camera.py --kernel 0
python3 red_shape_camera.py --s-min 70 --v-min 40 --hue-width 15
```

## 拍摄三个模型

先保持相机位置、光线、模型距离相近，各用正面和斜面测试，再改变距离。建议一次只放一根柱子，完整露出顶部和底部，背景尽量没有红色物体。

```bash
python3 red_shape_camera.py --label low5
python3 red_shape_camera.py --label normal10
python3 red_shape_camera.py --label tall15
```

每个命令分别运行，换模型时先退出上一条。按 **S** 保存当前帧的四个文件到程序旁的 `captures/`：

1. `_raw.png`：原始图像，未改变颜色或添加标注。
2. `_mask.png`：本帧红色检测掩膜。
3. `_view.png`：带测量标注和面板的预览。
4. `_metrics.json`：本帧各目标的尺寸、比例、轮廓指标及当前阈值。

按 **Q / Esc** 或 **Ctrl+C** 退出。给三个模型分别拍一些照片后，可以用这些数值比较实际比例区间。

也可以分析已有照片，保留照片原始分辨率，不会拉伸为 640×480：

```bash
python3 red_shape_camera.py --image model.jpg --label normal10
python3 red_shape_camera.py --image model.jpg --no-window
```

## 依赖与验证范围

实时模式只直接依赖 `opencv-python`（或系统 OpenCV）、NumPy、DonkeyCar 及其 PiCamera 后端。照片模式不需要 DonkeyCar。与参考脚本一样，GUI 需要可用桌面显示。

代码中的 `detect_red(rgb, settings)` 可独立调用；输入必须是 RGB 图像。它输出掩膜和候选目标列表，便于以后接入已有竞赛程序。

已通过 14 项自动检查：用模拟 RGB 图像检查红/蓝通道、红色色相跨界、不同尺寸和倾斜、分离目标、暗红/噪点、边缘截断，以及原图保存；相机参数与 DonkeyCar 图结构使用替身对象验证。已实际运行 `--image --no-window` 入口并检查生成的预览和 JSON。验证环境为 Python 3.12、OpenCV 5.0、NumPy 2.3；采用的轮廓/HSV API 也属于 OpenCV 4 系列已有接口，但此处没有在你们树莓派的具体版本上实测，也没有实测实时 GUI。

在装有 OpenCV、NumPy 的环境里，可以复跑检查（不打开真实相机）：

```bash
python3 -m unittest -v test_red_shape_camera
```

技术参考：

- [DonkeyCar 摄像头实现](https://github.com/autorope/donkeycar/blob/main/donkeycar/parts/camera.py)
- [OpenCV HSV 颜色分割](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html)
- [OpenCV 轮廓和包围框](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
- [Picamera2 手册：相机控制和像素格式](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)
