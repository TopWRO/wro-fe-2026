import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0)
print("Opened", ser.name)

buffer = ""
timer1 = time.time()

ser.write(b"reset gyro\n")
print("[PY] reset gyro sent")

try:
    while True:
        # 读出当前所有待收字节
        n = ser.in_waiting
        if n:
            chunk = ser.read(n).decode('utf-8', errors='ignore')
            if chunk:
                buffer += chunk

                # 按行拆分
                *lines, buffer = buffer.split('\n')  # buffer 中留最后一段（可能是不完整行）

                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith('#'):
                        # 注释行
                        # print("COMMENT:", line)
                        continue

                    parts = line.split(',')
                    if len(parts) != 7:
                        # print("BAD LINE:", repr(line))
                        continue

                    try:
                        angle, front, FrontRight, RightRear, back, LeftRear, FrontLeft = map(float, parts)
                        print(f"{angle:7.2f} {front:7.1f} {FrontRight:7.1f} {RightRear:7.1f} {back:7.1f} {LeftRear:7.1f} {FrontLeft:7.1f}")
                    except ValueError:
                        # print("PARSE ERR:", repr(line))
                        pass

        # 每 10 秒发一次 reset gyro 命令
        if time.time() > (timer1 + 10):
            timer1 = time.time()


        time.sleep(0.01)  # 小小 sleep 防止 100% 占 CPU

except KeyboardInterrupt:
    print("\n[PY] end")
    ser.close()
