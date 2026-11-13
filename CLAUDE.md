# CLAUDE.md — Top WRO FE 2026

## 项目概况
- 本项目用于参加 **WRO 2026 Future Engineers**（自动驾驶小车赛项）。
- GitHub 仓库：https://github.com/TopWRO/wro-fe-2026 （远程名 `origin`，主分支 `main`）
- 本地路径：`/home/pi/Top WRO FE 2026`（**路径含空格**，在 shell 命令里必须加引号）
- 自动驾驶部分基于 **DonkeyCar** 开源框架，用深度学习训练模型完成驾驶。

## 硬件
| 部件 | 型号 | 作用 |
|---|---|---|
| 主控 | Raspberry Pi 5 | 运行 DonkeyCar、摄像头采集、模型推理 |
| 协处理器 | Raspberry Pi Pico 2 | 读取传感器，与 Pi 5 通信 |
| 距离传感器 | 6 × ToF（前 F 为 VL53L4CX，R1/R2/B/L2/L1 为 VL53L4CD） | 接在 Pico 2 的 I2C1（GP2/GP3），XSHUT 分别为 GP20/21/18/19/16/17，地址 0x2A–0x2F |
| 陀螺仪 | BNO085 | 接在 Pico 2 的 UART0（GP0/GP1，115200，RVC 帧），提供朝向/角度 |
| 颜色传感器 | TCS34725（芯片 ID 0x44；旧版固件注释亦写明） | 接在 Pico 2 的 I2C0（GP4/GP5，地址 0x29），补光 LED 在 GP6 |
| 电机 | 2 × DYNAMIXEL XL330-M288-T，经 U2D2（`/dev/ttyUSB0`，57600，Protocol 2.0） | ID 1 = 驱动（Velocity 模式，电机 1 圈 → 车轮 3 圈）；ID 2 = 转向（Position 模式，中位 2016，±40°） |
| 人机界面 | Sense HAT | 摇杆确认 CW/CCW、LED 状态（比赛程序仍在使用） |

## Pi 5 ↔ Pico 通信
- Pico 固件为 MicroPython，运行 `Pi Pico Scripts/main.py`（需要同目录的 `sensor_hub_tof.py` 和 ToF 驱动；`vl53l4cd_patch (1).py` 在 Pico 上必须叫 `vl53l4cd_patch.py`），USB 串口 `/dev/ttyACM0`，115200。
- Pico 每 50 ms 输出一行：`degree=..,F=..,R1=..,R2=..,B=..,L2=..,L1=..,C1R=..,C1G=..,C1B=..,C1C=..`；ToF 单位 cm，`-1`=无有效读数，`-2`=已关闭；`degree=nan`=陀螺仪无数据。以 `#` 开头的是状态/错误信息。
- 命令（一行一条）：`reset`（航向归零）、`F:40` / `Front:40` / `R1:0` 等（ToF 测量时间 ms，0=关闭，10..200）、`stats`（10 秒 ToF 刷新率统计）。
- 默认每个 ToF 40 ms、轮流测量，所以每个 ToF 实际约 3 Hz 更新（输出 20 Hz 但数值会重复）。
- 测试脚本：`python pico_hub_test.py`（先设置 ToF 测量时间再持续监视，运行中可输入命令；要改测量时间就改 `main()` 里 `set_tof_time(F=40, ...)` 这一行）、`--test`（自动测试命令）、`--soft-reset`（重启 Pico 上的 main.py）。
- 比赛库 `AFI_CUP_V05.py`（障碍赛）/ `AFI_CUP_V05f.py`（开放赛）的 `UnifiedRobotIO` 已适配当前固件：按字段名解析；ToF 的 -1/-2 统一成 -1（下游代码只判断 `!= -1`）；`degree=nan` → `angle_valid=False` 并保留上次角度；角度不取反（`angle = degree`；当前固件 CW 为正、CCW 为负，与比赛流程目标角度一致；旧代码的取反会导致 CW 时 `ai_run` 永远到不了停止角度）；颜色存为 `C1R/C1G/C1B/C1C`；发送 `\nreset\n` 后丢弃数据帧，直到收到 `# OK reset`（1 秒超时并警告）。

## 文档仓库
- 2026 开源文档仓库在 Mac 本地 `~/Top_WRO-2026-documentation`（计划 GitHub：`TopWRO/Top_WRO-2026-documentation`）。代码是从本仓库原样复制并只加了英文注释；改了这里的比赛代码后，文档仓库的 `src/` 也要同步。

<!-- TODO：补充摄像头型号 -->

## Surprise Rule（障碍赛附加任务，规则版本 2026-11-09）
- 规则文件：`WRO-2026-Future-Engineers-Extra-Task.pdf`（在 Mac 的 `~/Downloads`，没有放进仓库）。
- **场地**：障碍赛正常随机化之后，裁判随机选一个柱子（不会是行驶方向上遇到的第一个），换成**相反颜色**的柱子，高度随机为 **50 mm 或 150 mm**（标准 100 mm），底面 50×50 mm 和位置不变。同一轮所有参加的队伍位置和高度相同。
- **行驶要求**：这个柱子要按**原来的颜色**通过（原红 → 从右侧过，虽然看到的是绿；原绿 → 从左侧过，虽然看到的是红）。其他柱子按标准规则。
- **可选参加**：每次正式尝试前告诉裁判是否参加，开始后不能改。不参加就是标准场地和标准计分。
- **计分**：每次正确通过 +5 分，最多 3 次共 +15 分（障碍赛满分 77，总分满分 137）；没跑完三圈也保留已得分；走错边按现有规则结束本轮；有维修动作则总分 ÷2。
- **现有问题**：`ai_run` 的模型只学过 100 mm 柱子，按看到的颜色走，遇到改过的柱子会走错边。
- **Potential solution（2026-11-12 讨论，尚未实现、尚未验证）**：利用"改过的柱子高度异常"这一点，逻辑是"高度正常按可见颜色走，高度异常按相反颜色走"。分两部分：
  - 训练：数据里加入 50 mm / 150 mm 的柱子，但**仍按可见颜色的正常方向走**，让模型对柱子高度不敏感。
  - 比赛：在 `ai_run` 里缩放和模型推理之间（`AFI_CUP_V05.py` 的 `center_crop` 和 `ai_pilot.run` 之间）加一步图像处理：HSV 找红/绿色块 → 在 320×240 原图上用色块高宽比判断高度 → 高度异常就把色块涂成相反颜色再送进模型；连续几帧确认后锁定，直到柱子离开画面；Run 脚本里加"参加 / 不参加"开关，不参加时流程与现在完全相同。
  - TODO（待实测）：三种高度的高宽比能否稳定区分（估计约 1:1 / 2:1 / 3:1，未测量）；换色后的图模型转向输出是否与正常柱子一致；品红停车挡板和橙线对红色识别的干扰；"参加 / 不参加"开关的设置方式是否符合启动规则（问裁判）。

## 运行环境
- 系统：Raspberry Pi OS 64 位，资源有限（CPU、内存、SD 卡速度都不高），写代码时注意性能，避免不必要的大依赖。
- Python 虚拟环境：`env`（终端提示符显示 `(env)`）。安装 Python 包前确认虚拟环境已激活，不要用 `sudo pip`。
<!-- TODO：补充虚拟环境的实际路径、DonkeyCar 版本、TensorFlow / PyTorch 版本 -->

## 版本控制规则（必须遵守）
1. **开始修改前**：运行 `git status`。如果有未提交的改动，先 commit 并 push，再开始新的修改。
2. **每完成一个独立的改动**（一个功能、一个修复、一次参数调整）：
   - `git add` 相关文件 → `git commit` → `git push`
   - 每次迭代都要 push，确保 GitHub 上随时有最新版本。
3. **提交信息**用英文，简短说明改了什么和为什么，例如：
   - `feat: add ToF wall-distance reading to Pico firmware`
   - `fix: correct BNO085 heading wrap-around at 360°`
   - `tune: lower max throttle for obstacle round`
4. **禁止**：`git push --force` / `-f`、`git reset --hard`、`git rebase` 改写已推送的历史、`rm -rf`。
5. **不要删除文件**，除非我明确要求。需要废弃的代码先告诉我。
6. 修改出问题时，用 `git revert` 或从历史中恢复，不要改写历史。

## 不要提交到 GitHub 的内容
GitHub 单文件上限 100MB，以下内容一律不提交（需要时加进 `.gitignore`）：
- 虚拟环境：`env/`、`venv/`
- DonkeyCar 采集的训练数据（tub 数据、图片）
- 训练好的大模型文件（`.h5`、`.keras`、`.tflite`、`.onnx` 等），除非我明确要求且文件很小
- 日志、缓存、录像
提交前用 `git status` 检查，发现大文件或数据文件要先问我。

## 安全规则
- **会让电机转动的程序，运行前必须先问我**：小车可能在桌上，突然动起来会摔坏。
- 不要修改 DYNAMIXEL 电机的 ID、波特率等 EEPROM 设置，除非我明确要求。
- 不要往 Pico 上刷固件，除非我明确要求。
- 修改 DonkeyCar 的 `myconfig.py` 时，在提交信息里写清楚改了哪些参数和原值。

## 工作方式
- 先读懂现有代码再改，优先做小而清晰的改动。
- 不确定硬件接线、通信协议或比赛规则时，先问我，不要猜。
- 改完后告诉我：改了什么、怎么测试、已经 push 的 commit 是哪个。

## 维护本文件（CLAUDE.md）
本文件是项目的"活文档"，你负责让它保持最新。

**什么时候更新：**
- 我提到新的硬件、换掉或移除了某个部件（传感器、电机、摄像头、电源等）
- 接线、引脚、通信方式、电机 ID、波特率等发生变化
- 软件环境变化：安装了新的库、升级了 DonkeyCar / TensorFlow 等版本、虚拟环境路径改变
- 项目结构变化：新增了重要的目录或脚本、改变了运行或训练的方式
- 我们做出了重要的设计决定（例如"障碍赛改用 X 方案"），记录决定和原因
- 你从代码或我的回答中弄清了标为 `TODO` 的信息，就把它补上并删掉对应的 TODO

**怎么更新：**
- 只记录确定的事实。不确定的信息先问我，或者标为 `TODO` 而不是猜测。
- 保持简洁：更新原有条目，而不是在末尾不断追加；过时的信息直接改掉。
- **以下内容未经我明确同意不得删除或放宽**："版本控制规则"、"不要提交到 GitHub 的内容"、"安全规则"。
- CLAUDE.md 的修改单独提交，提交信息以 `docs:` 开头，例如 `docs: update CLAUDE.md with new camera model`，然后 push。
- 更新后用一两句话告诉我改了 CLAUDE.md 的哪些地方。
