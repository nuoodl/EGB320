#!/usr/bin/env python3
"""
Sequence: servo -> 97 deg, drive forward 60mm, servo -> 40 deg.

Talks to the Nano over USB serial using the S<angle> / OK / ERR protocol
implemented in nano_servo_control.ino. Requires pyserial:
    pip install pyserial
"""

import serial
import time
import sys

# Genuine Nanos (ATmega328 + FTDI/CH340 depending on clone) usually show up
# as /dev/ttyUSB0; some show as /dev/ttyACM0. Run `ls /dev/tty*` before and
# after plugging in to see which one appears.
NANO_PORT = "/dev/ttyUSB0"
BAUD = 115200


def connect_nano():
    nano = serial.Serial(NANO_PORT, BAUD, timeout=5)
    # The Nano resets when the serial connection opens (DTR toggle) — give
    # it time to boot and print READY before sending anything, or your
    # first command can land during reset and get silently dropped.
    time.sleep(2)
    nano.reset_input_buffer()
    return nano


def set_servo(nano, angle):
    nano.write(f"S{angle}\n".encode())
    response = nano.readline().decode().strip()
    if response == "":
        raise RuntimeError("No response from Nano (timed out) — check wiring/port/baud")
    if response != "OK":
        raise RuntimeError(f"Nano rejected servo command: {response!r}")


def move_forward_mm(distance_mm):
    """
    Placeholder — fill in with your actual encoder motor driving code.

    This needs, at minimum: wheel diameter (or drive mechanism geometry),
    encoder pulses-per-revolution, and however you're actually driving the
    motors (GPIO PWM into an H-bridge? a dedicated motor driver board?).
    Tell me what hardware you're using here and I'll write this properly
    instead of leaving it stubbed.
    """
    raise NotImplementedError("Encoder motor control not yet implemented")


def main():
    nano = connect_nano()
    try:
        print("Setting servo to 97 degrees...")
        set_servo(nano, 97)

        print("Driving forward 60mm...")
        move_forward_mm(60)

        print("Setting servo to 40 degrees...")
        set_servo(nano, 40)

        print("Sequence complete.")
    except Exception as e:
        print(f"Failed: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        nano.close()


if __name__ == "__main__":
    main()
