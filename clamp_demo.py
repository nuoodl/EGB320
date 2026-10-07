"""
Test the Pi <-> ESP32 clamp link on its own. No motors, no camera.

    python3 clamp_test.py          ping, raise, lower, collect, release
    python3 clamp_test.py ping     just check the link

Pi UART must be enabled: sudo raspi-config -> Interface Options -> Serial Port
  login shell over serial: NO,  serial hardware enabled: YES, then reboot.
"""
import sys
import time

import serial

PORTS = ["/dev/serial0", "/dev/ttyUSB0", "/dev/ttyACM0"]
BAUD = 115200


def open_link():
    for p in PORTS:
        try:
            ser = serial.Serial(p, BAUD, timeout=0.5)
            time.sleep(0.5)
            ser.reset_input_buffer()
            print(f"open on {p}")
            return ser
        except Exception:
            continue
    sys.exit("no serial port found")


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()


def ask(ser, cmd, timeout):
    ser.reset_input_buffer()
    send(ser, cmd)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            return line
    return None


if __name__ == "__main__":
    ser = open_link()
    print("PING    ->", ask(ser, "PING", 2.0))
    if "ping" not in sys.argv:
        print("RAISE   (idle, 40 deg)");   send(ser, "RAISE");  time.sleep(1.5)
        print("LOWER   (open, 95 deg)");   send(ser, "LOWER");  time.sleep(1.5)
        print("COLLECT ->", ask(ser, "COLLECT", 8.0))
        time.sleep(2.0)
        print("RELEASE ->", ask(ser, "RELEASE", 8.0))
        time.sleep(1.0)
        send(ser, "RAISE")
    ser.close()
