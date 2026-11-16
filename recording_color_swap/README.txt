带可选红绿替换的数据录制

程序：
  recording_n03_color_swap.py：基于 Recording_N02 的新录制入口。
  pillar_color_processor.py：基于原 320x240 双向颜色算法的可调用处理器，也可独立预览。

录制图像流：
  RGB 320x240 -> 可选红绿替换 -> resize 160x120 -> TubWriter
  PS4 的 user/angle、user/throttle、user/mode 与处理后的图像在同一循环保存。

开关：
  在 IDE 中打开 recording_n03_color_swap.py，顶部设置 ENABLE_COLOR_SWAP = True / False。
  保存后直接点击“运行”，不需要填写 --recording 或其他运行参数；默认进入手动录制模式。
  IDE 的解释器使用项目现有的 /home/pi/env/bin/python。
  录制开关保留当前用户设置。外部调用或可选命令行参数仍可覆盖该值；启动前确定开关。
  启用时沿用原模型，每帧检测并立即按本帧结果换色或保留原色。
  处理在 320x240 原图上完成；不向训练图像叠加框线、文字或调试图。
  关闭时不导入/创建颜色处理器，图像走原来的 resize 路径。
  开启的处理器持续逐帧跟踪；Tub 仍只在 abs(throttle) > 0.05 时记录。

贴边稳定性：
  在 pillar_color_processor.py 顶部可设置 SWAP_CONFIRM_FRAMES = 5。
  这个5帧门槛只用于授权左右离场保护，不是首次换色或恢复识别的等待门槛。
  普通检测每帧运行：本帧可靠判为tall/low就立即换色；本帧stand/unknown就保留原色。
  即使前面多帧未识别、手部遮挡过或追踪重建，后续首个可靠special结果仍立即换色。
  默认连续5次完整、可靠的tall/low结果赋予边缘保护资格，tall和low计同一换色动作。
  只有目标到了画面左侧或右侧，才启用历史保护来抵抗截断引起的分类抖动。
  完整视野的负证据或漏检会清除资格；恢复识别仍立即换色，边缘资格另行重新累计。
  单独的顶部/底部截断不使用历史判断强制换色，也不启用特殊填边。
  已获得资格的左右离场目标可继续处理当前可见1~2像素残条；没有当前目标时不画历史残影。
  超过6帧未匹配则清除旧轨迹。未获得资格的边缘目标仍用本帧分类，没有额外保持/填边。
  Rules.border_fill_pixels 默认4：只在左右保护激活且实际触边时补齐<=4像素浅锯齿。
  大缺口及当前图像中的相反颜色区域受到保护；小缺口的补齐允许有限扩展到灰暗边缘。
  可将5改为3或10调整边缘保护的确认速度；这不会改变普通检测立即换色的行为。
  几何追踪不能可靠区分同位置同颜色的瞬时替换/严重遮挡，仍需实景视频验证。
  独立 --image 图片诊断逐图分类；实时预览与录制使用上述逐帧分类+左右边缘保护逻辑。

树莓派运行（这些命令会启动真实摄像头、PS4 和电机）：
  cd "/home/pi/Top WRO FE 2026/recording_color_swap"
  /home/pi/env/bin/python recording_n03_color_swap.py --recording --color-swap
  /home/pi/env/bin/python recording_n03_color_swap.py --recording --no-color-swap
  如需关闭显示窗口，在命令末尾加 --no-window。

数据默认仍写项目根目录的 data，与原程序一致。
可使用 --data-path "/home/pi/Top WRO FE 2026/data_color_swap" 单独保存处理后的数据。
录制模式不再要求导入 Keras；--driving 保持原图路径并在需要时加载 Keras。

外部 Python 调用（项目根目录在 sys.path 中）：
  from recording_color_swap.recording_n03_color_swap import build_vehicle_recording
  vehicle = build_vehicle_recording(enable_color_swap=True, show_window=False)
  try:
      vehicle.start(rate_hz=20)
  finally:
      vehicle.stop()
  注意：build_vehicle_recording 本身就会打开真实硬件，不能在无准备时调用。

单独调用颜色处理（不操作硬件）：
  from recording_color_swap.pillar_color_processor import PillarColorProcessor
  processor = PillarColorProcessor()
  converted_rgb = processor.run(capture_rgb)
  输入/输出均为 uint8、RGB、shape=(240,320,3)。输入数组不被修改。
  每次录制创建独立实例；reset() 可清除追踪状态。
  模型和调色板已内嵌，默认不读取原目录或外部模型文件。

离线测试（从项目根目录执行，不操作真实硬件）：
  PYTHONDONTWRITEBYTECODE=1 /home/pi/env/bin/python -B -m unittest discover -s recording_color_swap/tests -v
  测试以模拟摄像头、PS4、电机和 Tub 验证整条图像/标签路径。

保留原 Recording_N02.py 和原 green_red_live_area_model_320x240.py 以便回溯与比对。
