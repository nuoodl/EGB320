"""
Interface check: Navigation -> Mobility
=======================================
Verifies that the velocities Navigation asks for produce the motion
Mobility actually delivers.

    python3 check_mobility.py --bench   wheels OFF the ground, safe first test
    python3 check_mobility.py --floor   wheels ON the ground, needs 1.5 m clear
    python3 check_mobility.py --speed   measure MAX_WHEEL_SPEED properly
    python3 check_mobility.py --deadband  find MIN_RAW properly

WHAT IT CHECKS
    1. Does drive(v, w) exist and accept the units nav sends
    2. Does a positive w turn LEFT (nav assumes it does)
    3. Do different commanded speeds produce visibly different speeds,
       or does everything collapse to one value through the deadband
    4. Is MAX_WHEEL_SPEED honest - every nav speed is scaled by it
    5. Where the motors actually stall

START WITH --bench. If the sign is backwards, much better to find that
with the wheels in the air.
"""

import os
import sys
import time

_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _name in sorted(os.listdir(_SYSTEM)):
    _path = os.path.join(_SYSTEM, _name)
    if os.path.isdir(_path) and not _name.startswith(("_", ".")):
        if _path not in sys.path:
            sys.path.insert(0, _path)

import rpm_control_olivew


def _check_interface():
    """Nav calls exactly these two. Fail loudly and early if they are
    missing, rather than mid-run."""
    missing = [n for n in ("drive", "stop") if not hasattr(mobility, n)]
    if missing:
        print(f"FAIL: mobility.py has no {', '.join(missing)}()")
        print("Navigation calls mobility.drive(v, w) and mobility.stop().")
        print("Without them nav crashes the moment it tries to move.")
        return False
    print("mobility.drive() and mobility.stop() present")
    return True


def _raw_for(v, w):
    """What raw values this v/w actually becomes - the conversion nav's
    commands go through."""
    wb = getattr(mobility, "WHEELBASE", 0.15)
    left = v - (w * wb / 2.0)
    right = v + (w * wb / 2.0)
    return mobility._to_raw(left), mobility._to_raw(right)


def bench():
    """Wheels off the ground. Confirms direction and that speeds differ."""
    if not _check_interface():
        return

    print("\nWHEELS OFF THE GROUND for this test.")
    input("Press Enter when the robot is safely propped up...")

    moves = [
        ("forward, explore speed", 0.045, 0.0,
         "both wheels forward, same speed"),
        ("forward, approach speed", 0.030, 0.0,
         "both forward, NOTICEABLY SLOWER than the last one"),
        ("forward, correcting speed", 0.022, 0.0,
         "both forward, slower again"),
        ("turn left", 0.0, 0.5,
         "LEFT wheel BACKWARD, RIGHT wheel FORWARD"),
        ("turn right", 0.0, -0.5,
         "LEFT forward, RIGHT backward"),
        ("arc left", 0.030, 0.4,
         "both forward, right faster than left"),
    ]

    try:
        for name, v, w, expect in moves:
            raw_l, raw_r = _raw_for(v, w)
            print(f"\n{name}:  v={v} m/s  w={w} rad/s   -> raw L={raw_l} R={raw_r}")
            print(f"  expect: {expect}")
            mobility.drive(v, w)
            time.sleep(2.5)
            mobility.stop()
            time.sleep(1.0)
    finally:
        mobility.stop()

    print("\n" + "=" * 58)
    print("If 'turn left' moved the robot RIGHT, the sign is inverted.")
    print("  -> negate TURN_GAIN in nav_hd_demo.py")
    print("If the three forward speeds looked IDENTICAL, they are all")
    print("clamping to the same raw value through the deadband.")
    print("  -> MAX_WHEEL_SPEED in mobility.py is wrong, run --speed")
    print("=" * 58)


def deadband():
    """Find the raw value where the motors genuinely start turning."""
    if not _check_interface():
        return

    print("\nWHEELS OFF THE GROUND.")
    print("Watch for the first value where they turn STEADILY,")
    print("not just twitch or buzz.\n")
    input("Press Enter to start...")

    try:
        for raw in range(10, 131, 10):
            print(f"  raw {raw}")
            mobility.motor_driver.set_raw_motor_speed(raw, raw)
            time.sleep(1.8)
    finally:
        mobility.motor_driver.set_raw_motor_speed(0, 0)

    print("\nPut the first value that turned steadily into MIN_RAW")
    print(f"in mobility.py (currently {getattr(mobility, 'MIN_RAW', '?')}).")


def speed():
    """Measure top speed so MAX_WHEEL_SPEED is real, not a guess."""
    if not _check_interface():
        return

    print("\nWHEELS ON THE GROUND. Needs about 1.5 m of clear run.")
    print("Mark the start position. Time nothing - just measure how far")
    print("it travels in the 3 seconds.\n")
    input("Press Enter, then it runs flat out after a countdown...")

    for n in (3, 2, 1):
        print(f"  {n}...")
        time.sleep(1)

    try:
        mobility.motor_driver.set_raw_motor_speed(127, 127)
        time.sleep(3.0)
    finally:
        mobility.motor_driver.set_raw_motor_speed(0, 0)

    print("\nStopped.")
    try:
        cm = float(input("How far did it travel, in cm? "))
    except ValueError:
        print("Not a number - run it again.")
        return

    measured = (cm / 100.0) / 3.0
    current = getattr(mobility, "MAX_WHEEL_SPEED", None)
    print("\n" + "=" * 58)
    print(f"Top speed = {measured:.4f} m/s")
    print(f"Set MAX_WHEEL_SPEED = {measured:.3f} in mobility.py")
    if current:
        print(f"(currently {current})")
        if abs(measured - current) / current > 0.2:
            print("That is a big change - every speed nav commands is")
            print("scaled by it, so re-run --bench afterwards.")

    min_raw = getattr(mobility, "MIN_RAW", 50)
    floor = min_raw / 127.0 * measured
    print(f"\nUsable speed band: {floor:.3f} to {measured:.3f} m/s")
    print("Nav speeds must sit inside that. Below the floor the wheels")
    print("do not move while the code believes it is driving.")
    print("=" * 58)


def floor_test():
    """On the ground. Does a commanded speed produce that speed."""
    if not _check_interface():
        return

    print("\nWHEELS ON THE GROUND. Needs about 1 m clear.")
    print("Measure how far it goes each time.\n")

    for v in (0.045, 0.030):
        input(f"Press Enter to drive at {v} m/s for 4 s...")
        try:
            mobility.drive(v, 0.0)
            time.sleep(4.0)
        finally:
            mobility.stop()
        try:
            cm = float(input("  distance travelled, cm: "))
        except ValueError:
            continue
        actual = (cm / 100.0) / 4.0
        print(f"  commanded {v:.3f} m/s -> actual {actual:.3f} m/s "
              f"(x{actual/v:.2f})\n")

    print("If actual is consistently off by the same factor,")
    print("MAX_WHEEL_SPEED needs that correction.")


if __name__ == "__main__":
    if "--bench" in sys.argv:
        bench()
    elif "--deadband" in sys.argv:
        deadband()
    elif "--speed" in sys.argv:
        speed()
    elif "--floor" in sys.argv:
        floor_test()
    else:
        print(__doc__)
