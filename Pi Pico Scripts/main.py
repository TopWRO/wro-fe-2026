from machine import Pin, UART, I2C, PWM
import time
import sys
import select


# ============================================================
# USER SETTINGS
# ============================================================

# ------------------------------------------------------------
# BNO085 UART-RVC
# ------------------------------------------------------------

# BNO085 UART TX ---> Pico GP1
# Pico UART0 RX    ---> GP1
BNO_RX_PIN = 17
BNO_TX_PIN = 0
BNO_BAUD = 115200


# ------------------------------------------------------------
# OUTPUT
# ------------------------------------------------------------

# Unified output frequency:
# 25 Hz = one line every 40 ms
OUTPUT_HZ = 25
OUTPUT_PERIOD_MS = 1000 // OUTPUT_HZ

# NOTE:
# Pico USB CDC print() output does not use a hardware baud-rate
# setting. On Raspberry Pi, open /dev/ttyACM0 at 115200.
OUTPUT_BAUD = 115200


# ------------------------------------------------------------
# LED heartbeat
# ------------------------------------------------------------

LED_HALF_PERIOD_MS = 500


# ------------------------------------------------------------
# YAW DIRECTION
# ------------------------------------------------------------

# Required direction:
# CW  = positive
# CCW = negative
CW_SIGN = 1.0


# ============================================================
# COLOR SENSOR SETTINGS
# ============================================================

# TCS34725:
# GP2 = SDA
# GP3 = SCL
# GP4 = Orange LED brightness control

I2C_SDA_PIN = 2
I2C_SCL_PIN = 3
COLOR_LED_PIN = 4

I2C_FREQ = 400000
TCS_ADDR = 0x29

# 0 = OFF
# 100 = Maximum
COLOR_LED_BRIGHTNESS = 100


# ============================================================
# TCS34725 REGISTERS
# ============================================================

COMMAND_BIT = 0x80

ENABLE  = 0x00
ATIME   = 0x01
CONTROL = 0x0F
ID_REG  = 0x12

CDATA = 0x14
RDATA = 0x16
GDATA = 0x18
BDATA = 0x1A

PON = 0x01
AEN = 0x02


# ============================================================
# TCS34725 SENSOR SETTINGS
# ============================================================

# 24 ms integration time
INTEGRATION_TIME = 0xF6

# Gain:
# 0x00 = 1x
# 0x01 = 4x
# 0x02 = 16x
# 0x03 = 60x
GAIN = 0x01


# ============================================================
# COLOR CLASSIFICATION THRESHOLDS
# ============================================================

# Based on your color_sensor_testv01.py real test data

NONE_C_THRESHOLD = 100

# Orange
ORANGE_RN_MIN = 0.39
ORANGE_RG_MIN = 1.10

# Purple
PURPLE_BN_MIN = 0.295
PURPLE_BR_MIN = 0.78


# ============================================================
# PICO ONBOARD LED
# ============================================================

led = Pin("LED", Pin.OUT)
led.value(0)

last_led_time = time.ticks_ms()


# ============================================================
# COLOR SENSOR LED PWM
# ============================================================

color_led_pwm = PWM(Pin(COLOR_LED_PIN))
color_led_pwm.freq(1000)


def set_color_led_brightness(percent):

    if percent < 0:
        percent = 0

    if percent > 100:
        percent = 100

    duty = int(percent * 65535 / 100)

    color_led_pwm.duty_u16(duty)


set_color_led_brightness(COLOR_LED_BRIGHTNESS)


# ============================================================
# UART0 - BNO085
# ============================================================

uart = UART(
    0,
    baudrate=BNO_BAUD,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(BNO_TX_PIN),
    rx=Pin(BNO_RX_PIN),
    rxbuf=512
)


# ============================================================
# I2C1 - TCS34725
# ============================================================

i2c = I2C(
    1,
    sda=Pin(I2C_SDA_PIN),
    scl=Pin(I2C_SCL_PIN),
    freq=I2C_FREQ
)


# ============================================================
# USB SERIAL INPUT
# ============================================================

# USB CDC input from Raspberry Pi / Thonny.
# Non-blocking so command checking does not stop sensor reading.

usb_poll = select.poll()
usb_poll.register(sys.stdin, select.POLLIN)


# ============================================================
# BNO085 RECEIVE BUFFER
# ============================================================

buffer = bytearray()


# ============================================================
# YAW VARIABLES
# ============================================================

raw_yaw = None
last_raw_yaw = None

continuous_yaw = 0.0
zero_offset = 0.0

yaw_initialized = False


# ============================================================
# TIMING
# ============================================================

last_output_time = time.ticks_ms()


# ============================================================
# BNO085: SIGNED INT16 LITTLE ENDIAN
# ============================================================

def int16_le(low_byte, high_byte):

    value = low_byte | (high_byte << 8)

    if value & 0x8000:
        value -= 65536

    return value


# ============================================================
# BNO085: PARSE RVC YAW
# ============================================================

def get_yaw(packet):

    if len(packet) != 19:
        return None

    # Header
    if packet[0] != 0xAA or packet[1] != 0xAA:
        return None

    # Checksum
    calculated_checksum = sum(packet[2:18]) & 0xFF

    if calculated_checksum != packet[18]:
        return None

    # Yaw bytes
    yaw_raw = int16_le(
        packet[3],
        packet[4]
    )

    # 0.01 degree / LSB
    yaw = yaw_raw * 0.01

    return yaw


# ============================================================
# BNO085: UPDATE CONTINUOUS YAW
# ============================================================

def update_continuous_yaw(new_yaw):

    global last_raw_yaw
    global continuous_yaw
    global yaw_initialized

    # First valid packet
    if not yaw_initialized:

        last_raw_yaw = new_yaw
        continuous_yaw = 0.0
        yaw_initialized = True

        return

    # Difference from previous raw yaw
    delta = new_yaw - last_raw_yaw

    # Unwrap +/-180 degree crossing
    if delta > 180.0:
        delta -= 360.0

    elif delta < -180.0:
        delta += 360.0

    # CW positive / CCW negative
    delta = delta * CW_SIGN

    # Accumulate
    continuous_yaw += delta

    last_raw_yaw = new_yaw


# ============================================================
# USB COMMAND
# ============================================================

def check_usb_command():

    global zero_offset

    events = usb_poll.poll(0)

    if events:

        ch = sys.stdin.read(1)

        if ch == "1":

            zero_offset = continuous_yaw

            # Keep command response separate from 25 Hz data lines.
            print("RESET=0")


# ============================================================
# TCS34725 LOW LEVEL FUNCTIONS
# ============================================================

def write_reg(reg, value):

    i2c.writeto_mem(
        TCS_ADDR,
        COMMAND_BIT | reg,
        bytes([value])
    )


def read_reg(reg):

    data = i2c.readfrom_mem(
        TCS_ADDR,
        COMMAND_BIT | reg,
        1
    )

    return data[0]


def read_u16(reg):

    data = i2c.readfrom_mem(
        TCS_ADDR,
        COMMAND_BIT | reg,
        2
    )

    return data[0] | (data[1] << 8)


# ============================================================
# COLOR CLASSIFICATION
# ============================================================

def classify_color(c, r, g, b):

    # Not enough reflected light / probably no ball
    if c < NONE_C_THRESHOLD:
        return "NONE"

    # Avoid divide-by-zero
    if c == 0 or g == 0 or r == 0:
        return "UNKNOWN"

    rn = r / c
    bn = b / c

    rg_ratio = r / g
    br_ratio = b / r

    # Orange
    if (
        rn > ORANGE_RN_MIN
        and
        rg_ratio > ORANGE_RG_MIN
    ):
        return "ORANGE"

    # Purple
    if (
        bn > PURPLE_BN_MIN
        and
        br_ratio > PURPLE_BR_MIN
    ):
        return "PURPLE"

    return "UNKNOWN"


# ============================================================
# READ COMPLETE COLOR DATA
# ============================================================

def read_color_data():

    c = read_u16(CDATA)
    r = read_u16(RDATA)
    g = read_u16(GDATA)
    b = read_u16(BDATA)

    # Normalized RGB
    if c > 0:
        rn = r / c
        gn = g / c
        bn = b / c
    else:
        rn = 0.0
        gn = 0.0
        bn = 0.0

    # R/G ratio
    if g > 0:
        rg_ratio = r / g
    else:
        rg_ratio = 0.0

    # B/R ratio
    if r > 0:
        br_ratio = b / r
    else:
        br_ratio = 0.0

    color = classify_color(c, r, g, b)

    return (
        c,
        r,
        g,
        b,
        rn,
        gn,
        bn,
        rg_ratio,
        br_ratio,
        color
    )


# ============================================================
# STARTUP INFORMATION
# ============================================================

print()
print("============================================================")
print("Pico BNO085 + TCS34725 Unified Sensor")
print("============================================================")
print("BNO085 UART:")
print("  GP1 = RX from BNO085 TX")
print("  Baud = {}".format(BNO_BAUD))
print()
print("TCS34725:")
print("  GP2 = SDA")
print("  GP3 = SCL")
print("  GP4 = LED PWM")
print("  Address = 0x29")
print("  Integration = 24 ms")
print("  Gain = 4x")
print("  Color LED = {}%".format(COLOR_LED_BRIGHTNESS))
print()
print("Unified USB output:")
print("  Output = {} Hz".format(OUTPUT_HZ))
print("  Period = {} ms".format(OUTPUT_PERIOD_MS))
print("  Raspberry Pi serial setting = {} baud".format(OUTPUT_BAUD))
print()
print("Yaw:")
print("  CW  = positive")
print("  CCW = negative")
print("  USB command '1' = reset yaw to zero")
print("============================================================")
print()


# ============================================================
# COLOR SENSOR: I2C SCAN
# ============================================================

devices = i2c.scan()

print("I2C devices:", [hex(x) for x in devices])

if TCS_ADDR not in devices:

    print()
    print("*** ERROR ***")
    print("TCS34725 not found at 0x29")

    while True:
        led.toggle()
        time.sleep_ms(200)


# ============================================================
# COLOR SENSOR: CHIP ID
# ============================================================

chip_id = read_reg(ID_REG)

print("TCS34725 Chip ID = 0x{:02X}".format(chip_id))


# ============================================================
# COLOR SENSOR: INITIALIZATION
# ============================================================

write_reg(ENABLE, PON)

time.sleep_ms(3)

write_reg(ATIME, INTEGRATION_TIME)

write_reg(CONTROL, GAIN)

write_reg(
    ENABLE,
    PON | AEN
)

# Allow first color integration to complete
time.sleep_ms(30)


# ============================================================
# READY
# ============================================================

print()
print("Sensors ready.")
print()
print(
    "DATA FORMAT:"
)
print(
    "YAW=deg,C=clear,R=red,G=green,B=blue,"
    "rN=R/C,gN=G/C,bN=B/C,R/G=ratio,B/R=ratio,COLOR=result"
)
print()


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    now = time.ticks_ms()


    # ========================================================
    # 1. PICO LED HEARTBEAT
    # ========================================================

    if time.ticks_diff(now, last_led_time) >= LED_HALF_PERIOD_MS:

        led.toggle()

        last_led_time = now


    # ========================================================
    # 2. RECEIVE BNO085 UART DATA
    # ========================================================

    count = uart.any()

    if count > 0:

        incoming = uart.read(count)

        if incoming is not None:

            buffer.extend(incoming)


    # ========================================================
    # 3. PARSE ALL COMPLETE BNO085 RVC PACKETS
    # ========================================================

    while len(buffer) >= 19:

        # Find AA AA header
        if buffer[0] != 0xAA or buffer[1] != 0xAA:

            buffer = buffer[1:]
            continue

        packet = buffer[:19]

        yaw = get_yaw(packet)

        # Valid packet
        if yaw is not None:

            raw_yaw = yaw

            update_continuous_yaw(raw_yaw)

            buffer = buffer[19:]

        # Invalid packet
        else:

            buffer = buffer[1:]


    # ========================================================
    # 4. CHECK USB RESET COMMAND
    # ========================================================

    if yaw_initialized:

        check_usb_command()


    # ========================================================
    # 5. UNIFIED OUTPUT AT EXACTLY 25 Hz
    # ========================================================

    if time.ticks_diff(now, last_output_time) >= OUTPUT_PERIOD_MS:

        # Schedule next output based on actual current time.
        last_output_time = now

        # Only send unified sensor data after yaw is available.
        if yaw_initialized:

            # -----------------------------------------------
            # YAW FIRST
            # -----------------------------------------------

            output_yaw = continuous_yaw - zero_offset


            # -----------------------------------------------
            # COLOR SENSOR SECOND
            # -----------------------------------------------

            try:

                (
                    c,
                    r,
                    g,
                    b,
                    rn,
                    gn,
                    bn,
                    rg_ratio,
                    br_ratio,
                    color
                ) = read_color_data()


                # -------------------------------------------
                # ONE COMPLETE LINE
                #
                # YAW always comes first.
                # Color sensor detailed data follows.
                # -------------------------------------------

                print(
                    "YAW={:.2f},"
                    "C={},R={},G={},B={},"
                    "rN={:.3f},gN={:.3f},bN={:.3f},"
                    "R/G={:.3f},B/R={:.3f},"
                    "COLOR={}".format(
                        output_yaw,
                        c,
                        r,
                        g,
                        b,
                        rn,
                        gn,
                        bn,
                        rg_ratio,
                        br_ratio,
                        color
                    )
                )

            except Exception as e:

                # Keep YAW available even if color sensor has a temporary
                # communication problem.
                print(
                    "YAW={:.2f},COLOR_ERROR={}".format(
                        output_yaw,
                        e
                    )
                )


    # ========================================================
    # SMALL LOOP DELAY
    # ========================================================

    time.sleep_ms(1)
