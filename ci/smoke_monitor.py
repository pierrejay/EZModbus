"""Reset the board, print its console and exit with the smoke test verdict.

Usage: smoke_monitor.py <port> <timeout_s>
Exit code: 0 on "SMOKE PASS", 1 on "SMOKE FAIL", a crash, a reboot or a timeout.
"""
import sys
import time

import serial

CRASH_MARKERS = ("Guru Meditation", "abort() was called", "Backtrace:", "rst:0x", "Rebooting...")


def open_port(port, deadline):
    # The USB-Serial-JTAG port may disappear for a moment while the chip resets
    while True:
        try:
            return serial.Serial(port, 115200, timeout=0.2)
        except serial.SerialException:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.1)


def main():
    port, timeout = sys.argv[1], float(sys.argv[2])
    deadline = time.monotonic() + timeout
    ser = open_port(port, deadline)
    ser.dtr = False  # Boot strap released: normal boot
    ser.rts = True   # Pulse EN
    time.sleep(0.1)
    ser.rts = False

    started = False
    buf = b""
    while time.monotonic() < deadline:
        try:
            buf += ser.read(256)
        except serial.SerialException:
            ser.close()
            ser = open_port(port, deadline)
            continue
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").rstrip("\r")
            print(line, flush=True)
            if line.startswith("SMOKE START"):
                started = True
            elif started and line.startswith("SMOKE PASS"):
                return 0
            elif started and line.startswith("SMOKE FAIL"):
                return 1
            elif started and any(m in line for m in CRASH_MARKERS):
                print("SMOKE FAIL: crash or reboot", flush=True)
                return 1
    print("SMOKE FAIL: timeout" if started else "SMOKE FAIL: no output from the board", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
