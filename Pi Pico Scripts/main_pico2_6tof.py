# Pico 2 (RP2350) - 6 ToF + BNO085 RVC + TCS34725, 20 Hz
# Core0: existing six ToF strict round-robin, no changes to XSHUT/address/filter sequence.
# Core1: BNO085 UART-RVC (GP1), TCS34725 I2C0 (GP4/GP5), LED PWM (GP6), USB CSV 20 Hz.
# CSV: yaw,front,right,back,rear,left_rear,left_front,C,R,G,B
# Dependencies unchanged: vl53l4cd.py / vl53l4cx.py and both *_patch.py modules.
# IMPORTANT: sensor distances refresh roughly every 0.25-0.35 s, output repeats cached data at 20 Hz.

# tof_wt901_dualthread.py
# 结构：WT901 独立线程；主线程负责 ToF 初始化/严格轮询读取；USB CSV 目标输出频率20Hz。
# 注意：RP2040 MicroPython 只能新增一个线程（core1），因此仅 WT901 作为后台线程。
# 硬件：Pico；I2C1: GP2(SDA), GP3(SCL)；XSHUT: 前=GP20(L4CX), 右=GP21(L4CD)
#order: Yaw,Front,Right,Back,Rear,LeftRear,LeftFront,Clear,Red,Green,Blue

import time, _thread
from machine import Pin, I2C, UART, PWM
import sys
import select
import vl53l4cd_patch  # <<< 新增（必须）—让 VL53L4CD 支持 sigma/min_count 等门槛与读全量测量
import vl53l4cx_patch    # 前向 L4CX 的补丁（新建的这个）

try:
    import usb_cdc  # 需在 boot.py: usb_cdc.enable(console=True, data=True)
    _USB_VCP = getattr(usb_cdc, "data", None)
except Exception:
    usb_cdc = None
    _USB_VCP = None

# ========= 配置 =========
I2C_BUS, SDA_PIN, SCL_PIN = 1, 2, 3
COLOR_I2C_BUS, COLOR_SDA, COLOR_SCL = 0, 4, 5
COLOR_LED_PIN = 6
COLOR_LED_BRIGHTNESS = 100
COLOR_ADDR = 0x29
XSHUT_FRONT, XSHUT_RIGHT = 20, 21
XSHUT_BACK,  XSHUT_REAR   = 18, 19
XSHUT_LEFT_REAR, XSHUT_LEFT_FRONT = 16, 17

ADDR_DEFAULT = 0x29
ADDR_FRONT, ADDR_RIGHT = 0x2A, 0x2B
ADDR_BACK,  ADDR_REAR  = 0x2C, 0x2D
ADDR_LEFT_REAR, ADDR_LEFT_FRONT = 0x2E, 0x2F

TB_MS, IM_MS = 40, 0


UART_ID = 0
UART_BAUD = 115200
UART_TX, UART_RX = 0, 1

OUTPUT_HZ = 20
ECHO_TO_CONSOLE = False  # True 同时在 REPL 打印

# ========= 串口发送 =========

def send_line(s):
    line = (s + "\n").encode()
    if _USB_VCP is not None:
        try:
            _USB_VCP.write(line)
            if ECHO_TO_CONSOLE:
                sys.stdout.write(s + "\n")
            return
        except Exception:
            pass
    sys.stdout.write(s + "\n")

# ========= Shared thread state =========
led = Pin("LED", Pin.OUT)
lock = _thread.allocate_lock()
latest = {"yaw_deg": None, "tof": [-1.0] * 6, "color": None,
          "tof_ready": False, "color_error": None, "rvc_frames": 0}
ctrl = {"zero_requested": False}

# BNO085 UART-RVC frame: AA AA, index, yaw, pitch, roll,
# acceleration XYZ, checksum (19 bytes total).
def signed16(low, high):
    n = low | (high << 8)
    return n - 65536 if n & 0x8000 else n

def read_rvc_yaw(buffer):
    """Return (yaw_or_None, remaining_buffer), no bytearray deletion.

    Pico 2 MicroPython may not implement bytearray item deletion.
    Preserve incomplete packets for the next UART read.
    """
    pos = 0
    length = len(buffer)
    while length - pos >= 19:
        if buffer[pos] != 0xAA or buffer[pos + 1] != 0xAA:
            pos += 1
            continue
        packet = buffer[pos:pos + 19]
        if (sum(packet[2:18]) & 0xFF) != packet[18]:
            pos += 1
            continue
        return signed16(packet[3], packet[4]) * 0.01, buffer[pos + 19:]
    # Drop rejected header bytes but retain partial frame bytes.
    return None, buffer[pos:]

def color_write(i2c, reg, val):
    i2c.writeto_mem(COLOR_ADDR, 0x80 | reg, bytes([val]))

def color_read16(i2c, reg):
    b = i2c.readfrom_mem(COLOR_ADDR, 0x80 | reg, 2)
    return b[0] | (b[1] << 8)

def init_color():
    pwm = PWM(Pin(COLOR_LED_PIN))
    pwm.freq(1000)
    pwm.duty_u16(int(65535 * COLOR_LED_BRIGHTNESS / 100))
    i2c = I2C(COLOR_I2C_BUS, sda=Pin(COLOR_SDA), scl=Pin(COLOR_SCL), freq=100000)
    if COLOR_ADDR not in i2c.scan():
        return None, pwm, "NOT_FOUND_0x29"
    try:
        color_write(i2c, 0x00, 0x01)   # PON
        time.sleep_ms(3)
        color_write(i2c, 0x01, 0xF6)   # ~24 ms integration
        color_write(i2c, 0x0F, 0x01)   # 4x gain
        color_write(i2c, 0x00, 0x03)   # PON | AEN
        time.sleep_ms(30)
        return i2c, pwm, None
    except Exception as e:
        return None, pwm, repr(e)

def poll_usb_command():
    # Do not block the 20 Hz output while waiting for a full command line.
    try:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip().lower()
            if line in ("reset gyro", "reset_gyro", "gyro:reset", "1"):
                ctrl["zero_requested"] = True
    except Exception:
        pass

def sensor_and_output_thread():
    # This thread alone owns UART0, I2C0, and USB serial output.
    uart = UART(UART_ID, baudrate=UART_BAUD, tx=Pin(UART_TX), rx=Pin(UART_RX), rxbuf=1024)
    color_i2c, color_pwm, color_error = init_color()
    with lock:
        latest["color_error"] = color_error
    rvc_buffer = bytearray()
    raw_yaw = None
    yaw_zero = 0.0
    frames = 0
    color_data = None
    color_next = time.ticks_ms()
    interval = 1000 // OUTPUT_HZ
    next_output = time.ticks_add(time.ticks_ms(), interval)
    send_line("# BNO085_RVC,6TOF,TCS34725; output=20Hz; ToF strict round-robin")
    send_line("# yaw_deg,front_cm,right_cm,back_cm,rear_cm,left_rear_cm,left_front_cm,C,R,G,B")
    while True:
        n = uart.any()
        if n:
            data = uart.read(n)
            if data:
                rvc_buffer.extend(data)
                if len(rvc_buffer) > 512:
                    rvc_buffer = rvc_buffer[-38:]
                while True:
                    yaw, rvc_buffer = read_rvc_yaw(rvc_buffer)
                    if yaw is None:
                        break
                    raw_yaw = yaw
                    frames += 1
        if ctrl["zero_requested"]:
            if raw_yaw is not None:
                yaw_zero = raw_yaw
            ctrl["zero_requested"] = False
        now = time.ticks_ms()
        if color_i2c is not None and time.ticks_diff(now, color_next) >= 0:
            try:
                color_data = (color_read16(color_i2c, 0x14),
                              color_read16(color_i2c, 0x16),
                              color_read16(color_i2c, 0x18),
                              color_read16(color_i2c, 0x1A))
                color_error = None
            except Exception as e:
                color_data = None
                color_error = repr(e)
            color_next = time.ticks_add(now, interval)
        if time.ticks_diff(now, next_output) >= 0:
            poll_usb_command()
            yaw_val = None if raw_yaw is None else raw_yaw - yaw_zero
            with lock:
                latest["yaw_deg"] = yaw_val
                latest["color"] = color_data
                latest["color_error"] = color_error
                latest["rvc_frames"] = frames
                tof_values = latest["tof"][:]
            # NA indicates missing IMU/color input, not a fake zero reading.
            yaw_text = "NA" if yaw_val is None else "%.2f" % yaw_val
            tof_text = ["%.1f" % v for v in tof_values]
            color_text = ["NA"] * 4 if color_data is None else [str(v) for v in color_data]
            send_line(",".join([yaw_text] + tof_text + color_text))
            next_output = time.ticks_add(next_output, interval)
            if time.ticks_diff(now, next_output) >= 0:
                next_output = time.ticks_add(now, interval)
        time.sleep_ms(1)

# === 应用层可调阈值（先保证通，再逐步加严） ===
VALID_STATUS      = (4, 7, 9)

# 全局默认：给没特别指定的 L4CD 用（目前基本不用）
SIG_MIN_MCPS       = 0.12

# 每个方向单独的信号强度门槛
# 当前调试版本统一放宽到 0.05，便于观察 front/right/back/left 的真实读数。
SIG_MIN_FRONT_MCPS = 0.01    # front: VL53L4CX 正前方
SIG_MIN_RIGHT_MCPS = 0.02    # right: 右前 L4CD
SIG_MIN_BACK_MCPS  = 0.02    # back: 右后 L4CD
SIG_MIN_REAR_MCPS       = 0.02    # rear: 车尾 L4CD (0x2D)
SIG_MIN_LEFT_REAR_MCPS  = 0.02    # left_rear: 左后 L4CD (0x2E)
SIG_MIN_LEFT_FRONT_MCPS = 0.02    # left_front: 左前 L4CD (0x2F)


AMB_MAX_MCPS      = 1.5
SIGMA_MAX_MM      = 45.0
RANGE_MIN_MM      = 10
RANGE_MAX_MM      = 1500

# ========= ToF debug output switches =========
# This debug version is for finding why a sensor outputs -1.
# It keeps normal CSV output, and also prints selected ToF diagnostics to Thonny console.
#
# DEBUG_TOF_ALL=True  : print all 6 sensors every DEBUG_PRINT_EVERY_N cycles.
# DEBUG_TOF_BAD_ONLY=True: print ONLY sensors listed in DEBUG_FOCUS_NAMES when they output -1.
DEBUG_TOF_ALL = False         # False: do not print all 6 sensors every cycle
DEBUG_TOF_BAD_ONLY = False     # True: print focus sensors when they become -1
DEBUG_PRINT_EVERY_N = 1       # 1=every cycle, 2=every 2 cycles, etc.

# Focus sensors for -1 debugging:
# front = front-facing VL53L4CX.
# Change to ("front", "right", "back") if you want to watch all key sensors.
DEBUG_FOCUS_NAMES = ("right", "back")# 


def _check_measurement(m, sig_min=SIG_MIN_MCPS):
    """
    Check one ToF measurement and return:
      ok: True/False
      reason: "OK" or failure reason string

    Important:
    This function does not change the measurement. It only tells us why
    it will be accepted or rejected by the application filter.
    """
    if m is None:
        return False, "NO_MEASUREMENT"

    status = m.get("range_status", 255)
    dcm = m.get("distance_cm", -999.0)
    dmm = dcm * 10.0
    sigma = m.get("sigma_mm", 9999.0)
    sig = m.get("signal_rate_mcps", -1.0)
    amb = m.get("ambient_rate_mcps", 9999.0)

    if status not in VALID_STATUS:
        return False, "BAD_STATUS status=%d allowed=%s" % (status, str(VALID_STATUS))

    if not (RANGE_MIN_MM <= dmm <= RANGE_MAX_MM):
        return False, "BAD_RANGE dmm=%.1f allowed=%d..%d" % (
            dmm, RANGE_MIN_MM, RANGE_MAX_MM
        )

    if sigma > SIGMA_MAX_MM:
        return False, "BAD_SIGMA sigma=%.2f max=%.2f" % (
            sigma, SIGMA_MAX_MM
        )

    if sig < sig_min:
        return False, "BAD_SIGNAL signal=%.3f min=%.3f" % (
            sig, sig_min
        )

    if amb > AMB_MAX_MCPS:
        return False, "BAD_AMBIENT ambient=%.3f max=%.3f" % (
            amb, AMB_MAX_MCPS
        )

    return True, "OK"


def _valid(m, sig_min=SIG_MIN_MCPS):
    """
    Backward-compatible True/False filter.
    Other old code can still call _valid() as before.
    """
    ok, reason = _check_measurement(m, sig_min)
    return ok


def _fmt_measurement(name, m, sig_min=SIG_MIN_MCPS):
    """
    Compact one-line formatting, kept for compatibility with older debug prints.
    """
    if m is None:
        return "%s: raw=None" % name

    ok, reason = _check_measurement(m, sig_min)

    return (
        "%s: ok=%d reason=%s "
        "d=%.1fcm status=%d sig=%.3f amb=%.3f sigma=%.2f sig_min=%.3f"
    ) % (
        name,
        1 if ok else 0,
        reason,
        m.get("distance_cm", -1.0),
        m.get("range_status", -1),
        m.get("signal_rate_mcps", -1.0),
        m.get("ambient_rate_mcps", -1.0),
        m.get("sigma_mm", -1.0),
        sig_min
    )


def _print_one_tof_pretty(name, m_raw, m_valid, reason, sig_min, out_val):
    """
    Pretty multi-line console output for one ToF sensor.
    This is easier to read in Thonny than one very long line.
    """
    ok_text = "YES" if (m_valid is not None) else "NO"

    if m_raw is None:
        print("  %-5s | OUT=%7.1f cm | OK=%-3s | reason=%s" %
              (name, out_val, ok_text, reason))
        print("        | raw=None | sig_min=%.3f" % sig_min)
        return

    dcm = m_raw.get("distance_cm", -1.0)
    dmm = dcm * 10.0
    status = m_raw.get("range_status", -1)
    sig = m_raw.get("signal_rate_mcps", -1.0)
    amb = m_raw.get("ambient_rate_mcps", -1.0)
    sigma = m_raw.get("sigma_mm", -1.0)

    print("  %-5s | OUT=%7.1f cm | OK=%-3s | reason=%s" %
          (name, out_val, ok_text, reason))
    print("        | raw_d=%7.1f cm (%7.1f mm) | status=%3d | signal=%7.3f MCPS | sig_min=%6.3f" %
          (dcm, dmm, status, sig, sig_min))
    print("        | ambient=%7.3f MCPS | sigma=%6.2f mm" %
          (amb, sigma))




def _print_pretty_sensor(name, m_raw, m_valid, reason, sig_min):
    """
    Backward-compatible pretty printer.
    This avoids NameError if older edited code calls _print_pretty_sensor().
    """
    out_val = -1.0 if (m_valid is None) else m_valid.get("distance_cm", -1.0)
    _print_one_tof_pretty(name, m_raw, m_valid, reason, sig_min, out_val)


def _print_tof_debug_block(cycle_id, yaw_deg,
                           rawF, mF, reasonF, f_val,
                           rawR, mR, reasonR, r_val,
                           rawB, mB, reasonB, b_val,
                           rawRear, mRear, reasonRear, rear_val,
                           rawLR, mLR, reasonLR, lr_val,
                           rawLF, mLF, reasonLF, lf_val):
    """
    Print one readable block for all 6 ToF sensors.
    """
    print("")
    print("========== TOF DEBUG cycle=%d yaw=%.2f ==========" % (cycle_id, yaw_deg))
    _print_one_tof_pretty("front",      rawF,    mF,    reasonF,    SIG_MIN_FRONT_MCPS,      f_val)
    _print_one_tof_pretty("right",      rawR,    mR,    reasonR,    SIG_MIN_RIGHT_MCPS,      r_val)
    _print_one_tof_pretty("back",       rawB,    mB,    reasonB,    SIG_MIN_BACK_MCPS,       b_val)
    _print_one_tof_pretty("rear",       rawRear, mRear, reasonRear, SIG_MIN_REAR_MCPS,       rear_val)
    _print_one_tof_pretty("left_rear",  rawLR,   mLR,   reasonLR,   SIG_MIN_LEFT_REAR_MCPS,  lr_val)
    _print_one_tof_pretty("left_front", rawLF,   mLF,   reasonLF,   SIG_MIN_LEFT_FRONT_MCPS, lf_val)

    print("  CSV: %.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f" %
          (yaw_deg, f_val, r_val, b_val, rear_val, lr_val, lf_val))
    print("================================================")

def _read_one(sensor, sig_min=SIG_MIN_MCPS):
    try:
        # 优先：带 read_measurement() 的（L4CD 打了补丁后也是这种）
        if hasattr(sensor, "read_measurement"):
            if hasattr(sensor, "data_ready") and (not sensor.data_ready):
                return None
            m = sensor.read_measurement()
            try:
                sensor.clear_interrupt()
            except:
                pass
            return m if _valid(m, sig_min) else None

        # 回退：L4CX 只有 get_distance()，自己拼一个“看上去合法”的测量
        d = sensor.get_distance()
        if d is None:
            return None
        m = dict(
            distance_cm=d,
            range_status=0,
            # 注意：这里也用 sig_min，而不是全局的 SIG_MIN_MCPS
            signal_rate_mcps=sig_min + 0.05,
            ambient_rate_mcps=0.0,
            sigma_mm=min(5.0, SIGMA_MAX_MM - 1.0)
        )
        return m if _valid(m, sig_min) else None
    except Exception:
        return None






# ========= 主线程：ToF 严格轮询 + 20Hz 最新缓存输出 =========

def main():
    """
    Version 2: Strict round-robin anti-interference ToF ranging.
    - All 6 ToF sensors are initialized and assigned addresses.
    - During normal reading, only ONE ToF is allowed to range at a time.
    - Each sensor is given 50 ms measurement time.
    - The active sensor is explicitly stopped before the next sensor starts.
    - Output format: yaw_deg,front_cm,right_cm,back_cm,rear_cm,left_rear_cm,left_front_cm

    Important:
    6 sensors x 40 ms = at least about 240 ms per full cycle, plus read/stop overhead.
    Therefore each direction is roughly 3-4 Hz, not 10 Hz.
    """
    STRICT_MEASURE_MS = 40
    STRICT_READY_TIMEOUT_MS = 8
    STRICT_GUARD_MS = 2

    i2c = I2C(I2C_BUS, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400_000)

    # Address-assignment stage:
    # Pull all XSHUT pins low first so only one default 0x29 sensor is present at a time.
    x_front      = Pin(XSHUT_FRONT,      Pin.OUT, value=0)
    x_right      = Pin(XSHUT_RIGHT,      Pin.OUT, value=0)
    x_back       = Pin(XSHUT_BACK,       Pin.OUT, value=0)
    x_rear       = Pin(XSHUT_REAR,       Pin.OUT, value=0)
    x_left_rear  = Pin(XSHUT_LEFT_REAR,  Pin.OUT, value=0)
    x_left_front = Pin(XSHUT_LEFT_FRONT, Pin.OUT, value=0)
    time.sleep_ms(10)

    from vl53l4cx import VL53L4CX
    from vl53l4cd import VL53L4CD

    # 1) Front: VL53L4CX, default 0x29 -> 0x2A.
    x_front.value(1)
    time.sleep_ms(50)
    front = VL53L4CX(i2c, address=ADDR_DEFAULT)
    try:
        front.set_address(ADDR_FRONT)
    except Exception:
        front._write_register(0x0001, bytes([ADDR_FRONT & 0x7F]))
    time.sleep_ms(5)
    front = VL53L4CX(i2c, address=ADDR_FRONT)

    # 2) Right: VL53L4CD, default 0x29 -> 0x2B.
    x_right.value(1)
    time.sleep_ms(50)
    right = VL53L4CD(i2c, address=ADDR_DEFAULT)
    time.sleep_ms(5)
    try:
        right.set_address(ADDR_RIGHT)
    except Exception:
        right._write_register(0x0001, bytes([ADDR_RIGHT & 0x7F]))
    time.sleep_ms(5)
    right = VL53L4CD(i2c, address=ADDR_RIGHT)

    # 3) Back: VL53L4CD, default 0x29 -> 0x2C.
    x_back.value(1)
    time.sleep_ms(50)
    back = VL53L4CD(i2c, address=ADDR_DEFAULT)
    time.sleep_ms(5)
    try:
        back.set_address(ADDR_BACK)
    except Exception:
        back._write_register(0x0001, bytes([ADDR_BACK & 0x7F]))
    time.sleep_ms(5)
    back = VL53L4CD(i2c, address=ADDR_BACK)

    # 4) Rear: VL53L4CD, default 0x29 -> 0x2D (car rear-facing).
    x_rear.value(1)
    time.sleep_ms(50)
    rear = VL53L4CD(i2c, address=ADDR_DEFAULT)
    time.sleep_ms(5)
    try:
        rear.set_address(ADDR_REAR)
    except Exception:
        rear._write_register(0x0001, bytes([ADDR_REAR & 0x7F]))
    time.sleep_ms(5)
    rear = VL53L4CD(i2c, address=ADDR_REAR)

    # 5) Left rear: VL53L4CD, GP16, default 0x29 -> 0x2E.
    x_left_rear.value(1)
    time.sleep_ms(50)
    left_rear = VL53L4CD(i2c, address=ADDR_DEFAULT)
    time.sleep_ms(5)
    try:
        left_rear.set_address(ADDR_LEFT_REAR)
    except Exception:
        left_rear._write_register(0x0001, bytes([ADDR_LEFT_REAR & 0x7F]))
    time.sleep_ms(5)
    left_rear = VL53L4CD(i2c, address=ADDR_LEFT_REAR)

    # 6) Left front: VL53L4CD, GP17, default 0x29 -> 0x2F.
    x_left_front.value(1)
    time.sleep_ms(50)
    left_front = VL53L4CD(i2c, address=ADDR_DEFAULT)
    time.sleep_ms(5)
    try:
        left_front.set_address(ADDR_LEFT_FRONT)
    except Exception:
        left_front._write_register(0x0001, bytes([ADDR_LEFT_FRONT & 0x7F]))
    time.sleep_ms(5)
    left_front = VL53L4CD(i2c, address=ADDR_LEFT_FRONT)

    sensors = (
        ("front",      front,      SIG_MIN_FRONT_MCPS),
        ("right",      right,      SIG_MIN_RIGHT_MCPS),
        ("back",       back,       SIG_MIN_BACK_MCPS),
        ("rear",       rear,       SIG_MIN_REAR_MCPS),
        ("left_rear",  left_rear,  SIG_MIN_LEFT_REAR_MCPS),
        ("left_front", left_front, SIG_MIN_LEFT_FRONT_MCPS),
    )

    # Configure all ToF sensors while stopped.
    # TB_MS is changed to 50 ms in this file.
    # IM_MS is changed to 0 so the sensor is in continuous-style mode while it is temporarily enabled.
    for name, s, sig_min in sensors:
        try:
            s.stop_ranging()
        except Exception:
            pass
        try:
            s.inter_measurement = IM_MS
        except Exception:
            pass
        try:
            s.timing_budget = TB_MS
        except Exception:
            pass
        try:
            s.set_sigma_limit_mm(SIGMA_MAX_MM)
            s.set_min_count_rate_mcps(sig_min)
        except Exception:
            pass
        try:
            s.stop_ranging()
        except Exception:
            pass

    def stop_quiet(sensor):
        """Stop ranging. This does not pull XSHUT low, so the assigned I2C address is preserved."""
        try:
            sensor.stop_ranging()
            return
        except Exception:
            pass
        try:
            sensor._write_register(0x0087, b"\x00")
            try:
                sensor._ranging = False
            except Exception:
                pass
        except Exception:
            pass

    def start_no_wait(sensor):
        """
        Start ranging without using driver start_ranging(), because the driver waits
        for data_ready and clears the first interrupt. Here we want a precise 50 ms
        measurement window controlled by this loop.
        """
        try:
            # 0x0087 is SYSTEM_START in the VL53L4CD-compatible driver.
            # b"\x21" is continuous mode when inter_measurement == 0.
            sensor._write_register(0x0087, b"\x21")
            try:
                sensor._ranging = True
            except Exception:
                pass
            return True
        except Exception:
            # Fallback. This may block longer, but still keeps only one sensor active.
            try:
                sensor.start_ranging()
                return True
            except Exception:
                return False

    def read_measurement_wait(name, sensor, sig_min, timeout_ms):
        """
        Wait briefly for data_ready, then read and validate one measurement.

        Return:
          m_raw: raw measurement dict, or None if no measurement was read
          m_valid: same dict if it passes filter, otherwise None
          fail_reason: "OK" or a detailed reason
        """
        try:
            if hasattr(sensor, "data_ready"):
                t0 = time.ticks_ms()
                while not sensor.data_ready:
                    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                        return None, None, "DATA_READY_TIMEOUT_%dms" % timeout_ms
                    time.sleep_ms(1)

            if hasattr(sensor, "read_measurement"):
                m = sensor.read_measurement()
                try:
                    sensor.clear_interrupt()
                except Exception:
                    pass

                ok, reason = _check_measurement(m, sig_min)
                if ok:
                    return m, m, "OK"
                return m, None, reason

            # Fallback path. Normally not used because vl53l4cd_patch and
            # vl53l4cx_patch add read_measurement().
            m = _read_one(sensor, sig_min=sig_min)
            if m is None:
                return None, None, "READ_ONE_NONE"

            ok, reason = _check_measurement(m, sig_min)
            if ok:
                return m, m, "OK"
            return m, None, reason

        except Exception as e:
            return None, None, "READ_EXCEPTION:%r" % e

    def measure_one_strict(name, sensor, sig_min):
        """
        Strict sequence:
        stop -> guard -> start this one sensor -> wait -> read -> stop -> guard.

        Return:
          m_raw: raw measurement dict
          m_valid: measurement dict only if it passes filters
          reason: OK or failure reason
        """
        stop_quiet(sensor)
        time.sleep_ms(STRICT_GUARD_MS)

        if not start_no_wait(sensor):
            stop_quiet(sensor)
            return None, None, "START_FAILED"

        time.sleep_ms(STRICT_MEASURE_MS)

        m_raw, m_valid, reason = read_measurement_wait(
            name, sensor, sig_min, STRICT_READY_TIMEOUT_MS
        )

        stop_quiet(sensor)
        time.sleep_ms(STRICT_GUARD_MS)
        return m_raw, m_valid, reason

    # Make absolutely sure all sensors are stopped before the first strict cycle.
    for _, s, _ in sensors:
        stop_quiet(s)
    time.sleep_ms(10)

    # All ToF init/measurement remains on core0, with I2C1 exclusively owned by this thread.
    # core1 simultaneously handles BNO085, color, and timed USB output.
    try:
        _thread.start_new_thread(sensor_and_output_thread, ())
    except OSError as e:
        send_line("# ERROR: second thread could not start: %r" % e)
        return

    # Original ToF startup messages are disabled to avoid mixing into 20Hz CSV.
    # send_line("# STRICT_RR_40MS_DEBUG: yaw_deg,front_cm,right_cm,back_cm,rear_cm,left_rear_cm,left_front_cm")
    # ============================================================
    # 20 Hz OUTPUT SCHEDULER
    #
    # ToF measurement behavior is unchanged:
    # - strict round-robin
    # - only ONE ToF ranges at a time
    # - same 40 ms active measurement window
    # - same ready timeout, guard time, filters and thresholds
    #
    # USB CSV output is decoupled from a full six-sensor round:
    # every ~50 ms, send the latest cached values.
    # ============================================================

    OUTPUT_PERIOD_MS = int(1000 / OUTPUT_HZ)

    f_val = -1.0
    r_val = -1.0
    b_val = -1.0
    rear_val = -1.0
    lr_val = -1.0
    lf_val = -1.0

    rawF = mF = None
    rawR = mR = None
    rawB = mB = None
    rawRear = mRear = None
    rawLR = mLR = None
    rawLF = mLF = None

    reasonF = reasonR = reasonB = reasonRear = reasonLR = reasonLF = "NOT_MEASURED_YET"

    sensor_steps = (
        ("front", front, SIG_MIN_FRONT_MCPS),
        ("right", right, SIG_MIN_RIGHT_MCPS),
        ("back", back, SIG_MIN_BACK_MCPS),
        ("rear", rear, SIG_MIN_REAR_MCPS),
        ("left_rear", left_rear, SIG_MIN_LEFT_REAR_MCPS),
        ("left_front", left_front, SIG_MIN_LEFT_FRONT_MCPS),
    )

    sensor_index = 0
    cycle_id = 0


    while True:
        name, sensor, sig_min = sensor_steps[sensor_index]
        m_raw, m_valid, reason = measure_one_strict(name, sensor, sig_min)

        if name == "front":
            rawF, mF, reasonF = m_raw, m_valid, reason
            f_val = -1.0 if mF is None else mF["distance_cm"]

        elif name == "right":
            rawR, mR, reasonR = m_raw, m_valid, reason
            r_val = -1.0 if mR is None else mR["distance_cm"]

        elif name == "back":
            rawB, mB, reasonB = m_raw, m_valid, reason
            b_val = -1.0 if mB is None else mB["distance_cm"]

        elif name == "rear":
            rawRear, mRear, reasonRear = m_raw, m_valid, reason
            rear_val = -1.0 if mRear is None else mRear["distance_cm"]

        elif name == "left_rear":
            rawLR, mLR, reasonLR = m_raw, m_valid, reason
            lr_val = -1.0 if mLR is None else mLR["distance_cm"]

        elif name == "left_front":
            rawLF, mLF, reasonLF = m_raw, m_valid, reason
            lf_val = -1.0 if mLF is None else mLF["distance_cm"]

        sensor_index += 1
        if sensor_index >= len(sensor_steps):
            sensor_index = 0
            cycle_id += 1

            with lock:
                y = latest.get("yaw_deg")
            if y is None:
                y = 0.0

            if DEBUG_TOF_ALL and (cycle_id % DEBUG_PRINT_EVERY_N == 0):
                _print_tof_debug_block(
                    cycle_id, y,
                    rawF, mF, reasonF, f_val,
                    rawR, mR, reasonR, r_val,
                    rawB, mB, reasonB, b_val,
                    rawRear, mRear, reasonRear, rear_val,
                    rawLR, mLR, reasonLR, lr_val,
                    rawLF, mLF, reasonLF, lf_val
                )

        with lock:
            latest["tof"] = [f_val, r_val, b_val, rear_val, lr_val, lf_val]
            latest["tof_ready"] = True

if __name__ == "__main__":
    main()
