# vl53l4cx.py —— VL53L4CX 的“最小可用”兼容层（基于现有 vl53l4cd.py）
# 目的：不改底层流程，先让 L4CX 跑起来；后续再按数据手册做专属初始化表优化。

from vl53l4cd import VL53L4CD as _Base
import struct

try:
    import i2c_device
except ImportError:
    raise RuntimeError("缺少 i2c_device.py，请将其置于 Pico 根目录。")

class VL53L4CX(_Base):
    """
    兼容层要点：
    - 不调用 _Base.__init__（绕过可能的型号校验），但复用其寄存器函数与初始化序列 _sensor_init()
    - 轻度放宽 Min Count Rate / Sigma，提高中远距稳定性
    - 保持与 L4CD 一致的 API：start_ranging()/stop_ranging()/get_distance() 返 cm
    """
    def __init__(self, i2c, address=0x29):
        # 为 _Base 的底层方法准备必要字段
        self._i2c = i2c
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        self._ranging = False

        # 直接复用底层初始化序列
        self._sensor_init()

        # —— 阈值微调（尽量温和，避免“过度放宽”导致噪声增大）——
        # 1) Min Count Rate Limit：寄存器 0x0066（9.7定点）。降至 ~0.25 MCPS。
        try:
            self._write_register(0x0066, struct.pack(">H", int(0.25 * (1 << 7))))
        except Exception:
            pass

        # 2) Sigma 阈值：寄存器 0x0064（14.2定点）。从约90mm放宽到 ~120mm。
        try:
            self._write_register(0x0064, struct.pack(">H", int(120 * 4)))
        except Exception:
            pass

    # —— 兼容性：部分第三方脚本习惯以 mm 使用，这里补一个便捷属性 ——
    @property
    def distance_mm(self) -> int:
        """返回毫米（基于 get_distance 的 cm 结果 ×10）。"""
        return int(self.get_distance() * 10)

    def set_address(self, new_address: int):
        """
        修改 I2C 地址（低7位有效），并重建 i2c_device。
        注意：修改地址前确保仅此一颗处于上电/释放XSHUT状态，避免地址冲突。
        """
        self._write_register(0x0001, struct.pack(">B", new_address & 0x7F))
        self.i2c_device = i2c_device.I2CDevice(self._i2c, new_address & 0x7F)

    # ——（可选）宽容的单位自检：若底层返回 mm，这里自动转成 cm，保证上层一致性 —— #
    def get_distance(self) -> float:
        """
        返回距离（单位：cm）。若底层返回的是 mm，这里自动转 cm；
        若底层已经是 cm，则直接返回。
        """
        val = super().get_distance()
        # 经验性判断：若值显著大于 200（>2m），可能是 mm；或手动做双读兜底
        if val > 200:  # 大概率是 mm
            return val / 10.0
        return float(val)
