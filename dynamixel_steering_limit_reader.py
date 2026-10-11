#!/usr/bin/env python3
import sys
import time
import threading
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

STEERING_ID = 2

ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132

TORQUE_DISABLE = 0

CURRENT_CENTER = 2013
UNITS_PER_REV = 4096
DEG_PER_UNIT = 360.0 / UNITS_PER_REV


def to_signed_32(value):
    if value & 0x80000000:
        value -= 0x100000000
    return value


def pos_to_deg_from_center(pos, center=CURRENT_CENTER):
    return (pos - center) * DEG_PER_UNIT


port = PortHandler(DEVICENAME)
packet = PacketHandler(PROTOCOL_VERSION)

if not port.openPort():
    print(f"[ERROR] Cannot open {DEVICENAME}")
    sys.exit(1)

if not port.setBaudRate(BAUDRATE):
    print(f"[ERROR] Cannot set baudrate {BAUDRATE}")
    port.closePort()
    sys.exit(1)

dxl_comm_result, dxl_error = packet.write1ByteTxRx(
    port, STEERING_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)

if dxl_comm_result != COMM_SUCCESS:
    print("[ERROR] Failed to disable torque:", packet.getTxRxResult(dxl_comm_result))
    port.closePort()
    sys.exit(1)

if dxl_error != 0:
    print("[ERROR] Dynamixel error:", packet.getRxPacketError(dxl_error))
    port.closePort()
    sys.exit(1)

print()
print("Steering torque DISABLED.")
print("Move the steering mechanism by hand.")
print()
print("Commands:")
print("  L + Enter : save current position as LEFT limit")
print("  R + Enter : save current position as RIGHT limit")
print("  C + Enter : save current position as manually chosen CENTER")
print("  S + Enter : show saved values and calculated center")
print("  Q + Enter : quit")
print()
print("Raw position 2047 is treated as 0 deg only for reference.")
print()

state = {
    "running": True,
    "latest_pos": None,
    "left": None,
    "right": None,
    "manual_center": None,
}


def reader():
    while state["running"]:
        pos, comm_result, dxl_error = packet.read4ByteTxRx(
            port, STEERING_ID, ADDR_PRESENT_POSITION
        )
        if comm_result == COMM_SUCCESS and dxl_error == 0:
            pos = to_signed_32(pos)
            state["latest_pos"] = pos
            deg = pos_to_deg_from_center(pos)
            print(
                f"\rPresent Position: {pos:6d}   Angle vs 2047: {deg:+8.2f} deg   ",
                end="",
                flush=True
            )
        else:
            print("\n[WARN] Read failed")
        time.sleep(0.05)


def show_results():
    print("\n")
    print("===== Saved values =====")
    print(f"LEFT  : {state['left']}")
    print(f"RIGHT : {state['right']}")
    print(f"Manual center: {state['manual_center']}")

    if state["left"] is not None and state["right"] is not None:
        mechanical_center = (state["left"] + state["right"]) / 2.0
        half_range_units = abs(state["right"] - state["left"]) / 2.0
        half_range_deg = half_range_units * DEG_PER_UNIT
        center_offset_units = mechanical_center - CURRENT_CENTER
        center_offset_deg = center_offset_units * DEG_PER_UNIT

        print()
        print(f"Calculated mechanical center : {mechanical_center:.1f}")
        print(f"Current configured center    : {CURRENT_CENTER}")
        print(
            f"Center difference            : {center_offset_units:+.1f} units "
            f"({center_offset_deg:+.2f} deg)"
        )
        print(
            f"Mechanical half-range        : {half_range_units:.1f} units "
            f"(about {half_range_deg:.2f} deg each side)"
        )
        print(
            f"Full raw range               : {abs(state['right'] - state['left'])} units "
            f"(about {abs(state['right'] - state['left']) * DEG_PER_UNIT:.2f} deg)"
        )
    print("========================")


thread = threading.Thread(target=reader, daemon=True)
thread.start()

try:
    while True:
        cmd = input("\nCommand [L/R/C/S/Q]: ").strip().lower()

        if cmd == "l":
            if state["latest_pos"] is not None:
                state["left"] = state["latest_pos"]
                print(f"Saved LEFT = {state['left']}")

        elif cmd == "r":
            if state["latest_pos"] is not None:
                state["right"] = state["latest_pos"]
                print(f"Saved RIGHT = {state['right']}")

        elif cmd == "c":
            if state["latest_pos"] is not None:
                state["manual_center"] = state["latest_pos"]
                print(f"Saved manual CENTER = {state['manual_center']}")

        elif cmd == "s":
            show_results()

        elif cmd == "q":
            break

finally:
    state["running"] = False
    time.sleep(0.1)

    print("\n")
    show_results()
    print("\nSteering torque remains DISABLED.")
    print("Power-cycle or re-enable torque in your normal robot program before driving.")
    port.closePort()
