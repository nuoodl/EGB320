#!/usr/bin/env python3
"""
Sequence: servo -> 97 deg, drive forward 60mm, servo -> 40 deg.

Talks to the Nano over USB serial using the S<angle> / OK / ERR protocol
implemented in nano_servo_control.ino. Requires pyserial:
    pip install pyserial
"""

import math
import serial
import time
import sys

from controller import Controller

# Genuine Nanos (ATmega328 + FTDI/CH340 depending on clone) usually show up
# as /dev/ttyUSB0; some show as /dev/ttyACM0. Run `ls /dev/tty*` before and
# after plugging in to see which one appears.
NANO_PORT = "/dev/ttyUSB0"
BAUD = 115200

# --- Drive calibration -----------------------------------------------------
# PLACEHOLDERS — measure/check these on your actual robot, they will not be
# right by default:
#   WHEEL_DIAMETER_MM  measure the wheel across the tread (not the hub).
#   COUNTS_PER_REV     encoder counts per one full wheel revolution — check
#                       your motor/gearbox/encoder datasheet, or the fast way:
#                       run test_motor_driving() and manually rotate one wheel
#                       exactly one turn, then read the printed tick delta.
WHEEL_DIAMETER_MM = 60.0
COUNTS_PER_REV = 360
TICKS_PER_MM = COUNTS_PER_REV / (math.pi * WHEEL_DIAMETER_MM)

DRIVE_SPEED = 80          # raw motor units, -127..127 — tune with
                          # test_motor_driving() first; too high and it lurches
                          # off target before the loop below can react
STRAIGHT_GAIN = 0.6       # correction strength per tick of left/right drift
MOVE_TIMEOUT_S = 10.0

motor_driver = Controller()


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


def move_forward_mm(distance_mm, speed=DRIVE_SPEED, timeout_s=MOVE_TIMEOUT_S):
    """
    Drive straight forward using motor_driver (your team's Controller —
    set_raw_motor_speed / get_encoder_ticks) until the average wheel travel
    reaches distance_mm, then stop.

    This is deliberately independent of nav_hd_demo.py's mobility/RescueLink
    stack — it talks to Controller directly, so this script runs standalone.

    Includes a simple proportional correction: cheap gear motors rarely spin
    at exactly the same rate under identical raw speed commands, so without
    this the robot drifts off a straight line over any real distance. Every
    loop tick it nudges each wheel's speed based on how far it's ahead of or
    behind the other, using their tick counts (not distance/time) as the
    reference — it's fighting motor mismatch, not measuring speed.
    """
    target_ticks = distance_mm * TICKS_PER_MM
    start_left, start_right = motor_driver.get_encoder_ticks()

    left_speed = speed
    right_speed = speed
    motor_driver.set_raw_motor_speed(left_speed, right_speed)
    start_time = time.time()

    try:
        while True:
            left, right = motor_driver.get_encoder_ticks()
            delta_left = left - start_left
            delta_right = right - start_right
            avg_delta = (delta_left + delta_right) / 2.0

            if avg_delta >= target_ticks:
                break

            if time.time() - start_time > timeout_s:
                raise RuntimeError(
                    f"move_forward_mm timed out after {timeout_s}s "
                    f"(reached {avg_delta:.0f}/{target_ticks:.0f} ticks) — "
                    f"check wiring, TICKS_PER_MM calibration, or the timeout"
                )

            # Straight-line correction: if left has gone further than right,
            # slow left down / speed right up, and vice versa.
            drift = delta_left - delta_right
            correction = STRAIGHT_GAIN * drift
            left_speed = max(-127, min(127, speed - correction))
            right_speed = max(-127, min(127, speed + correction))
            motor_driver.set_raw_motor_speed(left_speed, right_speed)

            time.sleep(0.02)
    finally:
        motor_driver.set_raw_motor_speed(0, 0)


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
