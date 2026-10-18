# vl53l4cx_patch.py —— 给 VL53L4CX 兼容层加功能：min_count/sigma/read_measurement（带容错常量 + status掩码）

import struct, time
import vl53l4cx as _m       # 关键：补丁基于你的 VL53L4CX 兼容层
VL = _m.VL53L4CX

# —— 与 L4CD 补丁一致：容错取寄存器常量，不存在就用标准地址兜底 —— #
def _C(name, default):
    return getattr(__import__("vl53l4cd"), name, default)  # 从 l4cd 常量里拿，拿不到就用 default

_R_RANGE_STATUS = _C("_VL53L4CD_RESULT_RANGE_STATUS",  0x0089)
_R_SIGNAL_RATE  = _C("_VL53L4CD_RESULT_SIGNAL_RATE",   0x008E)
_R_AMBIENT_RATE = _C("_VL53L4CD_RESULT_AMBIENT_RATE",  0x0090)
_R_SIGMA        = _C("_VL53L4CD_RESULT_SIGMA",         0x0092)
_R_DISTANCE     = _C("_VL53L4CD_RESULT_DISTANCE",      0x0096)
_R_MIN_CNT      = _C("_VL53L4CD_MIN_COUNT_RATE_RTN_LIMIT_MCPS", 0x0066)
_R_SIGMA_LIM    = _C("_VL53L4CD_RANGE_CONFIG_SIGMA_THRESH",     0x0064)

def _set_min_count_rate_mcps(self, mcps: float):
    if mcps < 0: mcps = 0.0
    raw = int(mcps * (1 << 7))  # 9.7 定点
    self._write_register(_R_MIN_CNT, struct.pack(">H", raw))

def _set_sigma_limit_mm(self, sigma_mm: int):
    if sigma_mm < 0: sigma_mm = 0
    raw = int(sigma_mm * 4)     # 14.2 定点
    self._write_register(_R_SIGMA_LIM, struct.pack(">H", raw))

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

VL.set_min_count_rate_mcps = _set_min_count_rate_mcps
VL.set_sigma_limit_mm      = _set_sigma_limit_mm
VL.read_measurement        = _read_measurement
