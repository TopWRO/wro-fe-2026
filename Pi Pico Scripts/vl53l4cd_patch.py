# vl53l4cd_patch.py
# 作用：为 VL53L4CD 类增加稳健功能：
# 1) 安全版 get_distance()（非递归+超时）
# 2) set_min_count_rate_mcps() / set_sigma_limit_mm() 两个门槛接口
# 3) read_measurement() 扩展测量数据
# 用法：主程序 import 本文件一次即可启用。

import struct, time
import vl53l4cd as _m
VL = _m.VL53L4CD

# —— 新增：容错获取常量（有的固件里这些名字可能不存在） ——
def _C(name, default):
    return getattr(_m, name, default)

# —— 新增：为关键寄存器定义“本地常量”，取不到就兜底用标准地址 ——
_R_RANGE_STATUS = _C("_VL53L4CD_RESULT_RANGE_STATUS", 0x0089)
_R_SIGNAL_RATE  = _C("_VL53L4CD_RESULT_SIGNAL_RATE",  0x008E)
_R_AMBIENT_RATE = _C("_VL53L4CD_RESULT_AMBIENT_RATE", 0x0090)
_R_SIGMA        = _C("_VL53L4CD_RESULT_SIGMA",        0x0092)
_R_DISTANCE     = _C("_VL53L4CD_RESULT_DISTANCE",     0x0096)




def _safe_get_distance(self, timeout_ms=200, wait_for_new_data=True, log_errors=False):
    t0 = time.ticks_ms()
    while True:
        try:
            if wait_for_new_data:
                while not self.data_ready:
                    if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                        return None
                    time.sleep_ms(1)
            dist = self.distance
            try: self.clear_interrupt()
            except Exception: pass
            return dist
        except OSError as err:
            if log_errors: print("[VL53L4CD] OSError:", err)
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                return None
            time.sleep_ms(2)

VL.get_distance = _safe_get_distance

def _set_min_count_rate_mcps(self, mcps: float):
    if mcps < 0: mcps = 0
    raw = int(mcps * (1 << 7))
    self._write_register(_m._VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS, struct.pack(">H", raw))

def _set_sigma_limit_mm(self, sigma_mm: int):
    if sigma_mm < 0: sigma_mm = 0
    raw = int(sigma_mm * 4)
    self._write_register(_m._VL53L4CD_RANGE_CONFIG_SIGMA_THRESH, struct.pack(">H", raw))

VL.set_min_count_rate_mcps = _set_min_count_rate_mcps
VL.set_sigma_limit_mm = _set_sigma_limit_mm

def _read_measurement(self):
    raw_status = self._read_register(_R_RANGE_STATUS, 1)[0]
    status = raw_status & 0x1F  # 只保留低5位

    sig_raw   = struct.unpack(">H", self._read_register(_R_SIGNAL_RATE,  2))[0]
    amb_raw   = struct.unpack(">H", self._read_register(_R_AMBIENT_RATE, 2))[0]
    sigma_raw = struct.unpack(">H", self._read_register(_R_SIGMA,        2))[0]
    dist_raw  = struct.unpack(">H", self._read_register(_R_DISTANCE,     2))[0]

    sig_mcps = sig_raw / 2048.0
    amb_mcps = amb_raw / 2048.0
    sigma_mm = sigma_raw / 4.0
    dist_cm  = dist_raw / 10.0
    return dict(
        distance_cm=dist_cm,
        range_status=status,
        signal_rate_mcps=sig_mcps,
        ambient_rate_mcps=amb_mcps,
        sigma_mm=sigma_mm
    )



def _dump_core_regs(self):
    raw_status = self._read_register(_R_RANGE_STATUS, 1)[0]
    sig_raw   = struct.unpack(">H", self._read_register(_R_SIGNAL_RATE,  2))[0]
    amb_raw   = struct.unpack(">H", self._read_register(_R_AMBIENT_RATE, 2))[0]
    sigma_raw = struct.unpack(">H", self._read_register(_R_SIGMA,        2))[0]
    dist_raw  = struct.unpack(">H", self._read_register(_R_DISTANCE,     2))[0]
    return {
        "status_raw": raw_status,
        "status_masked": raw_status & 0x1F,
        "signal_raw": sig_raw,   "signal_mcps": sig_raw/2048.0,
        "ambient_raw": amb_raw,  "ambient_mcps": amb_raw/2048.0,
        "sigma_raw": sigma_raw,  "sigma_mm": sigma_raw/4.0,
        "distance_raw": dist_raw,"distance_cm": dist_raw/10.0,
    }


def _wait_data_ready(self, timeout_ms=200):
    t0 = time.ticks_ms()
    while not self.data_ready:
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            return False
        time.sleep_ms(1)
    return True

VL.dump_core_regs = _dump_core_regs
VL.wait_data_ready = _wait_data_ready
VL.read_measurement = _read_measurement
