
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
| 距离传感器 | 6 × ToF | 接在 Pico 2 上，测量与墙/障碍物距离 |
| 陀螺仪 | BNO085 | 接在 Pico 2 上，提供朝向/角度 |
| 电机 | DYNAMIXEL XL330-M288-T | 驱动/转向 |

<!-- TODO：补充 Pi 与 Pico 的通信方式（USB 串口 / UART / I2C）、Pico 固件语言（MicroPython / C）、摄像头型号、各电机的 ID 和用途 -->

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
