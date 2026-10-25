#!/usr/bin/env python3
"""Pi 5 side test for the Pico sensor hub (Pi Pico Scripts/main.py).

EASY USE: edit the two lines in main() at the bottom, then run
    python pico_hub_test.py
While it runs you can also type a command + Enter (reset, F:100, stats ...).
Ctrl-C to stop.

The Pico prints one data line every 50 ms over USB serial:
    degree=..,F=..,R1=..,R2=..,B=..,L2=..,L1=..,C1R=..,C1G=..,C1B=..,C1C=..
plus status lines starting with "#". ToF values are in cm; -1 = no valid
reading, -2 = sensor disabled. degree=nan means no fresh IMU data.

Commands the Pico accepts (one per line):
    reset                      zero the heading (degree)
    F:40  Front:40  R1:0 ...   ToF timing budget in ms, 0 = off, 10..200
    stats                      10 s ToF update-rate report

Other modes:
    python pico_hub_test.py --test           # automatic command test, PASS/FAIL
    python pico_hub_test.py --test --skip-stats
    python pico_hub_test.py --soft-reset     # Ctrl-C + Ctrl-D: restart main.py first

Nothing here drives a motor; it only talks to the Pico.
"""

import math
import select
import sys
import time

import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

DATA_KEYS = ("degree", "F", "R1", "R2", "B", "L2", "L1",
             "C1R", "C1G", "C1B", "C1C")
TOF_KEYS = ("F", "R1", "R2", "B", "L2", "L1")


class PicoLink:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0, exclusive=True)
        self.ser.reset_input_buffer()  # drop old lines queued before we opened
        self.buffer = b""
        self.skip_partial = True  # first line after opening may be cut off

    def send(self, text):
        self.ser.write(text.encode("utf-8") + b"\n")
        print("[PI ->] %s" % text)

    def read_lines(self):
        waiting = self.ser.in_waiting
        if waiting:
            self.buffer += self.ser.read(waiting)
        parts = self.buffer.split(b"\n")
        self.buffer = parts.pop()  # keep the incomplete tail
        if self.skip_partial and parts:
            parts.pop(0)
            self.skip_partial = False
        lines = []
        for raw in parts:
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        return lines

    def soft_reset(self):
        """Stop main.py with Ctrl-C, then soft-reboot with Ctrl-D (reruns main.py)."""
        print("[PI] Ctrl-C: stopping main.py ...")
        self.ser.write(b"\x03\x03")
        deadline = time.time() + 7.0  # main.py waits up to 5 s for the ToF worker
        text = ""
        while time.time() < deadline and ">>>" not in text:
            for line in self.read_lines():
                print("RAW  %s" % line)
                text += line
            if self.buffer.endswith(b">>> "):
                text += ">>>"
            time.sleep(0.02)
        if ">>>" not in text:
            print("[PI] WARN no REPL prompt seen; sending Ctrl-D anyway")
        print("[PI] Ctrl-D: soft reboot, main.py restarts")
        self.buffer = b""
        self.ser.write(b"\x04")

    def close(self):
        self.ser.close()


def parse_data(line):
    """Return {key: float} for a data line, or None for anything else."""
    if line.startswith("#") or "=" not in line:
        return None
    values = {}
    for field in line.split(","):
        key, sep, text = field.partition("=")
        if not sep:
            return None
        try:
            values[key] = float(text)
        except ValueError:
            return None
    if tuple(values) != DATA_KEYS:
        return None
    return values


def tof_text(value):
    if value == -2:
        return "  off"
    if value < 0:
        return "   --"
    return "%5.1f" % value


def format_data(d):
    degree = "   nan" if math.isnan(d["degree"]) else "%+7.1f" % d["degree"]
    tof = " ".join("%s %s" % (k, tof_text(d[k])) for k in TOF_KEYS)
    return "deg %s | %s | RGBC %5d %5d %5d %5d" % (
        degree, tof, d["C1R"], d["C1G"], d["C1B"], d["C1C"])


def print_line(line):
    """Print one received line; return parsed data or None."""
    data = parse_data(line)
    if data is not None:
        print(format_data(data))
    elif line.startswith("#"):
        print("PICO %s" % line)
    else:
        print("RAW  %s" % line)  # REPL output, tracebacks, etc.
    return data


# --------------------------------------------------------------------------
# Easy helpers used by main()
# --------------------------------------------------------------------------

_link = None


def pico():
    """Open the Pico serial port once and reuse it."""
    global _link
    if _link is None:
        try:
            _link = PicoLink(PORT, BAUD)
        except serial.SerialException as exc:
            sys.exit("[PI] cannot open %s: %s (is another script using it?)" % (PORT, exc))
        print("[PI] opened %s" % PORT)
    return _link


def set_tof_time(**budgets_ms):
    """Send e.g. set_tof_time(F=100, R1=0) as "F:100" and "R1:0" to the Pico.

    Names: F, R1, R2, B, L2, L1. Value in ms: 0 = off, or 10..200.
    The Pico's "# OK ..." / "# ERR ..." replies show up in read_pico_forever().
    """
    for name, ms in budgets_ms.items():
        pico().send("%s:%s" % (name, ms))


def read_pico_forever(every=1):
    """Print data from the Pico until Ctrl-C; typed commands are sent to it.

    every=N prints only every Nth data line (1 = all, 20 lines/s).
    """
    link = pico()
    interactive = sys.stdin.isatty()
    if interactive:
        print("[PI] Monitoring. Type a command + Enter (e.g. reset, F:100, "
              "stats). Ctrl-C to quit.")
    count = 0
    window_start = time.time()
    window_count = 0
    try:
        while True:
            for line in link.read_lines():
                data = parse_data(line)
                if data is None:
                    print_line(line)
                    continue
                count += 1
                window_count += 1
                if count % every == 0:
                    print(format_data(data))

            if interactive and select.select([sys.stdin], [], [], 0)[0]:
                command = sys.stdin.readline().strip()
                if command:
                    link.send(command)

            now = time.time()
            if now - window_start >= 5.0:
                print("[PI] data rate %.1f lines/s" % (window_count / (now - window_start)))
                window_start = now
                window_count = 0
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[PI] stopped")
    finally:
        link.close()


# --------------------------------------------------------------------------
# Automatic test mode
# --------------------------------------------------------------------------

class Tester:
    def __init__(self, link):
        self.link = link
        self.results = []
        self.last_data = None

    def poll(self):
        """Read lines, print status lines, remember the newest data line."""
        lines = self.link.read_lines()
        for line in lines:
            data = parse_data(line)
            if data is None:
                print_line(line)
            else:
                self.last_data = data
        return lines

    def check(self, name, ok, detail=""):
        self.results.append((name, ok))
        print("  %s  %s%s" % ("PASS" if ok else "FAIL", name,
                              ("  (%s)" % detail) if detail else ""))

    def wait_for_line(self, predicate, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for line in self.poll():
                if predicate(line):
                    return line
            time.sleep(0.005)
        return None

    def wait_for_data(self, predicate, timeout):
        def is_match(line):
            data = parse_data(line)
            return data is not None and predicate(data)
        line = self.wait_for_line(is_match, timeout)
        return parse_data(line) if line else None

    def command(self, text, expect, timeout=1.5):
        """Send a command and expect a reply line containing `expect`."""
        print("\n--- send %r, expect %r" % (text, expect))
        self.link.send(text)
        reply = self.wait_for_line(lambda line: expect in line, timeout)
        self.check("%r -> %r" % (text, expect), reply is not None,
                   reply or "no reply within %.1f s" % timeout)
        return reply is not None

    def run(self, skip_stats):
        print("=== 1. receiving data")
        first = self.wait_for_data(lambda d: True, 8.0)
        self.check("data line received and parsed", first is not None,
                   "is main.py running? try --soft-reset" if first is None else "")
        if first is None:
            return False
        print("  " + format_data(first))

        print("\n=== 2. data rate (expect ~20 Hz)")
        start = time.time()
        count = 0
        while time.time() - start < 3.0:
            count += sum(1 for line in self.poll() if parse_data(line))
            time.sleep(0.005)
        rate = count / (time.time() - start)
        self.check("data rate 15..25 Hz", 15.0 <= rate <= 25.0, "%.1f Hz" % rate)

        print("\n=== 3. sensor values")
        d = self.last_data
        print("  " + format_data(d))
        self.check("IMU degree is a number", not math.isnan(d["degree"]),
                   "degree=%s" % d["degree"])
        bad = [k for k in TOF_KEYS if d[k] < 0]
        self.check("all 6 ToF have valid readings", not bad,
                   "no reading: %s" % ",".join(bad) if bad else "")
        self.check("color sensor readable", d["C1C"] >= 0, "C1C=%d" % d["C1C"])

        print("\n=== 4. valid commands")
        if self.command("reset", "# OK reset: degree=0.0", timeout=2.5):
            after = self.wait_for_data(lambda d: True, 1.0)
            self.check("degree near 0 after reset",
                       after is not None and abs(after["degree"]) < 2.0,
                       "degree=%s" % (after["degree"] if after else "none"))
        self.command("F:100", "# OK F=100 ms")
        self.command("Front:0", "# OK F disabled")
        off = self.wait_for_data(lambda d: d["F"] == -2.0, 1.0)
        self.check("data shows F=-2 (disabled)", off is not None)
        self.command("f: 40", "# OK F=40 ms")  # lower case + space are accepted
        on = self.wait_for_data(lambda d: d["F"] != -2.0, 1.0)
        self.check("data shows F enabled again", on is not None)

        print("\n=== 5. bad commands (Pico must reject them)")
        self.command("F: blabla", "# ERR Timing budget must be an integer")
        self.command("X:50", "# ERR Unknown direction")
        self.command("R1:5", "# ERR Use 0 (disabled), or 10..200 ms")
        self.command("R1:500", "# ERR Use 0 (disabled), or 10..200 ms")
        self.command("F", "# ERR Use Front:100 or R1:0")
        self.command("F:1:2", "# ERR Use Front:100 or R1:0")
        self.command("x" * 70, "# ERR command too long")
        alive = self.wait_for_data(lambda d: True, 1.0)
        self.check("still streaming data after bad commands", alive is not None)

        if not skip_stats:
            print("\n=== 6. stats (takes ~10 s)")
            if self.command("stats", "# STATS started"):
                done = self.wait_for_line(lambda l: "# STATS finished" in l, 13.0)
                self.check("stats report finished", done is not None)
                # Let the per-sensor STATS lines print.
                self.wait_for_line(lambda l: False, 0.5)
        return True

    def summary(self):
        passed = sum(1 for _, ok in self.results if ok)
        print("\n=== RESULT: %d/%d passed" % (passed, len(self.results)))
        for name, ok in self.results:
            if not ok:
                print("  FAIL  %s" % name)
        return passed == len(self.results)


def run_auto_test(skip_stats):
    link = pico()
    ok = False
    try:
        tester = Tester(link)
        tester.run(skip_stats)
        ok = tester.summary()
    except KeyboardInterrupt:
        print("\n[PI] stopped")
    finally:
        link.close()
    sys.exit(0 if ok else 1)


# --------------------------------------------------------------------------
# Edit these two lines to test
# --------------------------------------------------------------------------

def main():
    set_tof_time(F=40, R1=40, R2=40, B=40, L2=40, L1=40)  # ToF measuring time per sensor in ms (10..200, 0 = off); remove a name to leave that sensor unchanged
    read_pico_forever()  # keep receiving and printing data from the Pico; type a command + Enter to send it, Ctrl-C to stop


if __name__ == "__main__":
    if "--soft-reset" in sys.argv:
        pico().soft_reset()
    if "--test" in sys.argv:
        run_auto_test(skip_stats="--skip-stats" in sys.argv)
    else:
        main()
