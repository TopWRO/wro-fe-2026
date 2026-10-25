"""Pico/Pico 2 sensor hub: main USB/IMU/color thread + one ToF worker.

Run this entry file, with sensor_hub_tof.py and the five tested ToF drivers
in the Pico root directory. Normal data output is one labeled CSV line/50 ms.
"""

from machine import Pin, I2C, UART
import _thread
import time
import struct
import sys
import select

from sensor_hub_tof import parse_command, run_tof_worker

OUTPUT_PERIOD_MS = 50
IMU_TIMEOUT_MS = 2000
NAN = float("nan")


class HubShared:
    """Short locked copies only. No I2C, UART, USB or sleeps under this lock."""

    def __init__(self):
        self.lock = _thread.allocate_lock()
        self.tof = (-1.0,) * 6
        self.commands = []
        self.messages = []
        self.lost_messages = 0
        self.stop = False
        self.done = False
        self.failure = None

    def post_command(self, command):
        with self.lock:
            if len(self.commands) >= 32:
                return False
            self.commands.append(command)
            return True

    def take_commands(self):
        with self.lock:
            commands = self.commands
            self.commands = []
            return commands

    def message(self, text):
        with self.lock:
            if len(self.messages) < 64:
                self.messages.append(text)
            else:
                self.lost_messages += 1

    def take_messages(self):
        with self.lock:
            messages = self.messages[:8]
            self.messages = self.messages[len(messages):]
            lost = self.lost_messages
            self.lost_messages = 0
        if lost:
            messages.append("# WARN %d status messages dropped" % lost)
        return messages

    def publish(self, values):
        values = tuple(values)
        with self.lock:
            self.tof = values

    def snapshot(self):
        with self.lock:
            return self.tof, self.done, self.failure

    def should_stop(self):
        with self.lock:
            return self.stop

    def request_stop(self):
        with self.lock:
            self.stop = True

    def fail(self, reason):
        with self.lock:
            self.failure = reason

    def mark_done(self):
        with self.lock:
            self.done = True


class USBCommands:
    def __init__(self):
        self.poller = select.poll()
        self.poller.register(sys.stdin, select.POLLIN)
        self.line = ""
        self.overflow = False

    def read_lines(self):
        lines = []
        for _ in range(128):
            if not self.poller.poll(0):
                break
            char = sys.stdin.read(1)
            if not char:
                break
            if char in ("\n", "\r"):
                if self.overflow:
                    print("# ERR command too long (maximum 64 characters)")
                elif self.line.strip():
                    lines.append(self.line.strip())
                self.line = ""
                self.overflow = False
            elif not self.overflow:
                if len(self.line) < 64:
                    self.line += char
                else:
                    self.overflow = True
        return lines

    def close(self):
        self.poller.unregister(sys.stdin)


class ContinuousYaw:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = b""
        self.previous_yaw = None
        self.continuous_angle = 0.0
        self.zero_offset = 0.0
        self.last_reset_output = None
        self.last_valid_ms = None

    def poll(self):
        # Called at 20 Hz; decode every buffered frame, not just the last frame.
        available = self.uart.any()
        if available:
            data = self.uart.read(min(available, 2048))
            if data:
                self.buffer += data
        while len(self.buffer) >= 19:
            start = self.buffer.find(b"\xAA\xAA")
            if start < 0:
                self.buffer = self.buffer[-1:]
                break
            self.buffer = self.buffer[start:]
            if len(self.buffer) < 19:
                break
            if (sum(self.buffer[2:18]) & 0xFF) != self.buffer[18]:
                self.buffer = self.buffer[1:]
                continue
            yaw = struct.unpack_from("<h", self.buffer, 3)[0] / 100.0
            if self.previous_yaw is None:
                self.continuous_angle = yaw
            else:
                delta = (yaw - self.previous_yaw + 180.0) % 360.0 - 180.0
                self.continuous_angle += delta
            self.previous_yaw = yaw
            self.last_valid_ms = time.ticks_ms()
            self.buffer = self.buffer[19:]

    def fresh(self):
        return (self.last_valid_ms is not None and
                time.ticks_diff(time.ticks_ms(), self.last_valid_ms) <= IMU_TIMEOUT_MS)

    def reset(self):
        if not self.fresh():
            return False
        self.last_reset_output = self.continuous_angle - self.zero_offset
        self.zero_offset = self.continuous_angle
        return True

    def value(self):
        if not self.fresh():
            return NAN
        return self.continuous_angle - self.zero_offset


class ColorSensor:
    def __init__(self, i2c, led):
        self.i2c = i2c
        self.led = led
        self.address = 0x29
        if self.address not in i2c.scan():
            raise RuntimeError("Color sensor not found at I2C0 address 0x29")
        chip_id = i2c.readfrom_mem(self.address, 0x92, 1)[0]
        if chip_id != 0x44:
            raise RuntimeError("Unexpected color sensor ID: 0x%02X" % chip_id)
        self.write_reg(0x00, 0)
        self.write_reg(0x01, 0xF6)  # 24 ms integration, matching the passed test.
        self.write_reg(0x0F, 0x01)  # 4x gain.
        self.write_reg(0x00, 0x01)
        time.sleep_ms(3)
        self.write_reg(0x00, 0x03)
        time.sleep_ms(30)

    def write_reg(self, register, value):
        self.i2c.writeto_mem(self.address, 0x80 | register, bytes([value]))

    def read(self):
        data = self.i2c.readfrom_mem(self.address, 0xB4, 8)
        c, r, g, b = struct.unpack("<4H", data)
        return r, g, b, c

    def close(self):
        try:
            self.write_reg(0x00, 0)
        finally:
            self.led.value(0)


def format_output(degree, tof, color):
    return ("degree=%.1f,F=%.1f,R1=%.1f,R2=%.1f,B=%.1f,L2=%.1f,L1=%.1f,"
            "C1R=%d,C1G=%d,C1B=%d,C1C=%d" %
            ((degree,) + tuple(tof) + tuple(color)))


def main():
    shared = HubShared()
    uart = None
    color_sensor = None
    color_led = None
    commands = None
    worker_started = False
    reset_pending = False
    imu_error = False
    color_error = False
    try:
        uart = UART(0, baudrate=115200, bits=8, parity=None, stop=1,
                    tx=Pin(0), rx=Pin(1), rxbuf=2048, timeout=0)
        imu = ContinuousYaw(uart)
        color_led = Pin(6, Pin.OUT, value=1)
        color_i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=100_000)
        color_sensor = ColorSensor(color_i2c, color_led)
        commands = USBCommands()

        # Exactly one additional thread; main remains the sole USB owner.
        _thread.start_new_thread(run_tof_worker, (shared,))
        worker_started = True
        print("# Ready: two threads; USB/IMU/color 20 Hz; ToF default 40 ms")
        print("# Commands: reset; Front/R1/R2/Back/L2/L1:0 or 10..200; stats")
        next_output_ms = time.ticks_add(time.ticks_ms(), OUTPUT_PERIOD_MS)

        while True:
            for line in commands.read_lines():
                if line == "reset":
                    reset_pending = True
                    continue
                if line.lower() == "stats":
                    item = ("stats",)
                else:
                    try:
                        index, budget_ms = parse_command(line)
                    except ValueError as exc:
                        print("# ERR %s" % str(exc))
                        continue
                    item = ("budget", index, budget_ms)
                if not shared.post_command(item):
                    print("# ERR ToF command queue full; command not applied")

            for message in shared.take_messages():
                print(message)
            tof_values, worker_done, worker_failure = shared.snapshot()
            if worker_failure is not None:
                raise RuntimeError(worker_failure)
            if worker_done:
                raise RuntimeError("ToF worker stopped unexpectedly")

            now = time.ticks_ms()
            if time.ticks_diff(now, next_output_ms) >= 0:
                try:
                    imu.poll()
                    degree = imu.value()
                    missing = not imu.fresh()
                    if missing and not imu_error:
                        print("# ERR no fresh IMU data; degree=nan")
                    elif not missing and imu_error:
                        print("# INFO IMU data restored")
                    imu_error = missing
                    if reset_pending and imu.reset():
                        reset_pending = False
                        degree = 0.0
                        print("# OK reset: degree=0.0")
                except OSError as exc:
                    degree = NAN
                    if not imu_error:
                        print("# ERR IMU read: %s" % str(exc))
                    imu_error = True

                try:
                    color_values = color_sensor.read()
                    if color_error:
                        print("# INFO color data restored")
                    color_error = False
                except (OSError, ValueError) as exc:
                    color_values = (-1, -1, -1, -1)
                    if not color_error:
                        print("# ERR color read: %s" % str(exc))
                    color_error = True

                # Take a new immutable snapshot after UART/color reads.
                tof_values, worker_done, worker_failure = shared.snapshot()
                if worker_failure is not None or worker_done:
                    raise RuntimeError(worker_failure or "ToF worker stopped unexpectedly")
                print(format_output(degree, tof_values, color_values))
                next_output_ms = time.ticks_add(next_output_ms, OUTPUT_PERIOD_MS)
                if time.ticks_diff(time.ticks_ms(), next_output_ms) >= 0:
                    next_output_ms = time.ticks_add(time.ticks_ms(), OUTPUT_PERIOD_MS)
            time.sleep_ms(1)

    except KeyboardInterrupt:
        pass

    finally:
        shared.request_stop()
        try:
            if worker_started:
                stop_started_ms = time.ticks_ms()
                while not shared.snapshot()[1]:
                    if time.ticks_diff(time.ticks_ms(), stop_started_ms) > 5000:
                        print("# ERR ToF worker still stopping; soft-reset Pico before rerunning")
                        break
                    time.sleep_ms(5)
        finally:
            if commands is not None:
                commands.close()
            try:
                if color_sensor is not None:
                    color_sensor.close()
                elif color_led is not None:
                    color_led.value(0)
            finally:
                if uart is not None:
                    uart.deinit()


if __name__ == "__main__":
    main()
