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
  启用时沿用原模型，但需连续确认后才换色，详见下面“贴边稳定性”。
  处理在 320x240 原图上完成；不向训练图像叠加框线、文字或调试图。
  关闭时不导入/创建颜色处理器，图像走原来的 resize 路径。
  开启的处理器持续逐帧跟踪；Tub 仍只在 abs(throttle) > 0.05 时记录。

贴边稳定性：
  在 pillar_color_processor.py 顶部可设置 SWAP_CONFIRM_FRAMES = 5。
  默认连续5次完整、可靠的 tall / low 检测确认“需要换色”；tall与low均投同一个换色动作。
  未确认时保留原色；正常、未知、截断或漏检会打断未完成的确认。
  确认后对同一目标锁定换色，近处变大、左右截断或后续分类抖动不再反复切换。
  同色几何追踪支持画面边缘的窄片段，当前可见1~2像素的残条也可继续处理。
  未检测到物体的帧不画历史掩膜；超过6帧未匹配则清除旧轨迹，新目标重新确认。
  Rules.border_fill_pixels 默认4：对已确认且实际触边的目标补齐<=4像素浅锯齿。
  大缺口及当前图像中的相反颜色区域受到保护；小缺口的补齐允许有限扩展到灰暗边缘。
  若需更快确认可设为3，更长确认可设为10；确认越长，首次换色越晚。
  仅在边缘首次出现、没有完整视野确认历史的目标不会被强制换色。
  几何追踪不能可靠区分同位置同颜色的瞬时替换/严重遮挡，仍需实景视频验证。
  独立 --image 图片诊断仍逐图分类；实时预览与录制使用上述连续确认/锁定逻辑。

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
