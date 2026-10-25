"""ToF core derived from the hardware-tested controller; owned by core 1."""

from machine import Pin, I2C
import time
import vl53l4cd_patch
import vl53l4cx_patch
from vl53l4cd import VL53L4CD
from vl53l4cx import VL53L4CX
DEFAULT_BUDGET_MS = 40
READY_EXTRA_MS = 8
SWITCH_GUARD_MS = 4
VALID_RAW_STATUS = (4, 7, 9)
LAYOUT = (('F', 20, 42, VL53L4CX, 0.01), ('R1', 21, 43, VL53L4CD, 0.02), ('R2', 18, 44, VL53L4CD, 0.02), ('B', 19, 45, VL53L4CD, 0.02), ('L2', 16, 46, VL53L4CD, 0.02), ('L1', 17, 47, VL53L4CD, 0.02))
NAMES = {'front': 0, 'f': 0, 'r1': 1, 'r2': 2, 'back': 3, 'b': 3, 'l2': 4, 'l1': 5}

def parse_command(line):
    if line.count(':') != 1:
        raise ValueError('Use Front:100 or R1:0; one command per line')
    name, text = line.split(':', 1)
    name = name.strip().lower()
    if name not in NAMES:
        raise ValueError('Unknown direction; use Front,R1,R2,Back,L2,L1')
    try:
        budget_ms = int(text.strip())
    except ValueError:
        raise ValueError('Timing budget must be an integer in milliseconds')
    if budget_ms != 0 and (not 10 <= budget_ms <= 200):
        raise ValueError('Use 0 (disabled), or 10..200 ms')
    return (NAMES[name], budget_ms)

def start_no_wait(sensor):
    sensor.clear_interrupt()
    sensor._write_register(135, b'!')
    sensor._ranging = True

class MeasurementStats:
    """Timestamp completed new reads, independently of cached USB output."""

    def __init__(self, emit):
        self.emit = emit
        self.active = False

    def start(self, budgets):
        self.budgets = list(budgets)
        self.count = [0] * 6
        self.valid_count = [0] * 6
        self.first = [None] * 6
        self.last = [None] * 6
        self.valid_first = [None] * 6
        self.valid_last = [None] * 6
        self.minimum = [None] * 6
        self.maximum = [None] * 6
        self.timeouts = [0] * 6
        self.errors = [0] * 6
        self.started_ms = time.ticks_ms()
        self.active = True
        self.emit('# STATS started: 10 seconds; keep sensor settings unchanged')

    def cancel(self):
        if self.active:
            self.active = False
            self.emit('# STATS cancelled: settings changed; send stats again')

    def record(self, index, valid):
        if not self.active:
            return
        now = time.ticks_ms()
        if self.first[index] is None:
            self.first[index] = now
        else:
            interval = time.ticks_diff(now, self.last[index])
            if self.minimum[index] is None or interval < self.minimum[index]:
                self.minimum[index] = interval
            if self.maximum[index] is None or interval > self.maximum[index]:
                self.maximum[index] = interval
        self.last[index] = now
        self.count[index] += 1
        if valid:
            if self.valid_first[index] is None:
                self.valid_first[index] = now
            self.valid_last[index] = now
            self.valid_count[index] += 1

    def failure(self, index, timeout=False):
        if self.active:
            if timeout:
                self.timeouts[index] += 1
            else:
                self.errors[index] += 1

    def finish_if_due(self):
        if not self.active:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self.started_ms)
        if elapsed < 10000:
            return
        self.active = False
        self.emit('# STATS finished: window_ms=%d' % elapsed)
        for index, row in enumerate(LAYOUT):
            label = row[0]
            if self.budgets[index] == 0:
                self.emit('# STATS %s disabled' % label)
                continue
            count = self.count[index]
            valid_count = self.valid_count[index]
            if count >= 2:
                span = time.ticks_diff(self.last[index], self.first[index])
                average = span / (count - 1)
                update_hz = 1000.0 * (count - 1) / span if span > 0 else 0.0
                timing = 'avg_ms=%.1f update_hz=%.3f' % (average, update_hz)
            else:
                timing = 'avg_ms=NA update_hz=NA'
            if valid_count >= 2:
                valid_span = time.ticks_diff(self.valid_last[index], self.valid_first[index])
                valid_hz = 1000.0 * (valid_count - 1) / valid_span if valid_span > 0 else 0.0
                valid_text = '%.3f' % valid_hz
            else:
                valid_text = 'NA'
            self.emit('# STATS %s %s valid_hz=%s samples=%d valid=%d' % (label, timing, valid_text, count, valid_count))
            if count >= 2:
                self.emit('# STATS %s min_ms=%d max_ms=%d timeouts=%d errors=%d' % (label, self.minimum[index], self.maximum[index], self.timeouts[index], self.errors[index]))
            else:
                self.emit('# STATS %s timeouts=%d errors=%d' % (label, self.timeouts[index], self.errors[index]))

class ToFController:

    def __init__(self, sensors, emit):
        self.sensors = sensors
        self.budgets = [DEFAULT_BUDGET_MS] * 6
        self.values = [-1.0] * 6
        self.last_status = [None] * 6
        self.last_error = ['NOT_MEASURED'] * 6
        self.stats = MeasurementStats(emit)
        self.cursor = 0
        self.active = None
        self.started_ms = 0
        self.active_budget_ms = 0
        self.next_start_ms = time.ticks_add(time.ticks_ms(), SWITCH_GUARD_MS)

    def stop_active(self):
        if self.active is not None:
            self.sensors[self.active].stop_ranging()
            self.cursor = (self.active + 1) % 6
            self.active = None
            self.next_start_ms = time.ticks_add(time.ticks_ms(), SWITCH_GUARD_MS)

    def set_budget(self, index, budget_ms):
        if not 0 <= index < 6:
            raise ValueError('Invalid sensor index')
        if budget_ms != 0 and (not 10 <= budget_ms <= 200):
            raise ValueError('Use 0 (disabled), or 10..200 ms')
        if self.budgets[index] == budget_ms:
            return
        self.stats.cancel()
        if self.active == index:
            self.stop_active()
        if budget_ms > 0:
            self.sensors[index].timing_budget = budget_ms
        self.budgets[index] = budget_ms
        self.values[index] = -2.0 if budget_ms == 0 else -1.0
        self.last_status[index] = None
        self.last_error[index] = 'DISABLED' if budget_ms == 0 else 'NOT_MEASURED'

    def step(self):
        if self.active is None:
            if time.ticks_diff(time.ticks_ms(), self.next_start_ms) < 0:
                return
            for offset in range(6):
                index = (self.cursor + offset) % 6
                if self.budgets[index] > 0:
                    start_no_wait(self.sensors[index])
                    self.active = index
                    self.active_budget_ms = self.budgets[index]
                    self.started_ms = time.ticks_ms()
                    break
            return
        index = self.active
        sensor = self.sensors[index]
        elapsed = time.ticks_diff(time.ticks_ms(), self.started_ms)
        if elapsed < self.active_budget_ms:
            return
        finished = False
        try:
            if sensor.data_ready:
                measurement = sensor.read_measurement()
                status = measurement['range_status']
                distance_cm = measurement['distance_cm']
                self.last_status[index] = status
                if status in VALID_RAW_STATUS and distance_cm >= 0:
                    self.values[index] = distance_cm
                    self.last_error[index] = 'OK'
                else:
                    self.values[index] = -1.0
                    self.last_error[index] = 'INVALID_STATUS_OR_DISTANCE'
                sensor.clear_interrupt()
                self.stats.record(index, status in VALID_RAW_STATUS and distance_cm >= 0)
                finished = True
            elif elapsed >= self.active_budget_ms + READY_EXTRA_MS:
                self.values[index] = -1.0
                self.last_status[index] = None
                self.last_error[index] = 'READY_TIMEOUT'
                self.stats.failure(index, timeout=True)
                finished = True
        except (OSError, ValueError, KeyError) as exc:
            self.values[index] = -1.0
            self.last_status[index] = None
            self.last_error[index] = str(exc)
            self.stats.failure(index)
            finished = True
        if finished:
            self.stop_active()

def initialize_sensors(i2c, xshut, sensors, emit, should_stop):
    for index, row in enumerate(LAYOUT):
        if should_stop():
            return False
        label, gpio, address, driver, min_signal = row
        try:
            xshut[index].value(1)
            time.sleep_ms(50)
            if 41 not in i2c.scan():
                raise RuntimeError('No default-address device at 0x29')
            sensor = driver(i2c, address=41)
            sensors.append(sensor)
            if should_stop():
                return False
            sensor.stop_ranging()
            sensor.set_address(address)
            time.sleep_ms(5)
            sensor.inter_measurement = 0
            sensor.timing_budget = DEFAULT_BUDGET_MS
            sensor.set_sigma_limit_mm(45)
            sensor.set_min_count_rate_mcps(min_signal)
            sensor.clear_interrupt()
            sensor.stop_ranging()
            if address not in i2c.scan():
                raise RuntimeError('Assigned address not detected')
            emit('# INIT %s OK: XSHUT=GP%d, address=0x%02X' % (label, gpio, address))
        except Exception as exc:
            raise RuntimeError('INIT %s failed: %s' % (label, str(exc)))
    return True


def run_tof_worker(shared):
    """Only this worker accesses I2C1/XSHUT; all messages go to core 0."""
    sensors = []
    xshut = []
    try:
        for row in LAYOUT:
            xshut.append(Pin(row[1], Pin.OUT, value=0))
        time.sleep_ms(10)
        if shared.should_stop():
            return
        i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400_000)
        if not initialize_sensors(i2c, xshut, sensors, shared.message, shared.should_stop):
            return
        controller = ToFController(sensors, shared.message)
        previous_values = tuple(controller.values)
        shared.publish(previous_values)
        shared.message("# ToF ready: all six directions enabled, 40 ms each")

        while not shared.should_stop():
            for command in shared.take_commands():
                if shared.should_stop():
                    break
                if command[0] == "stats":
                    controller.stats.start(controller.budgets)
                else:
                    index, budget_ms = command[1], command[2]
                    controller.set_budget(index, budget_ms)
                    label = LAYOUT[index][0]
                    if budget_ms == 0:
                        shared.message("# OK %s disabled" % label)
                    else:
                        shared.message("# OK %s=%d ms" % (label, budget_ms))

            if shared.should_stop():
                break
            controller.step()
            controller.stats.finish_if_due()
            values = tuple(controller.values)
            if values != previous_values:
                shared.publish(values)
                previous_values = values
            time.sleep_ms(1)

    except Exception as exc:
        shared.fail("ToF worker: %s" % str(exc))

    finally:
        try:
            for sensor in sensors:
                try:
                    sensor.stop_ranging()
                except Exception:
                    pass
            for pin in xshut:
                pin.value(0)
        finally:
            shared.mark_done()
