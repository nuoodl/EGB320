"""
Interface check: Vision -> Navigation
=====================================
Verifies that what Vision hands Navigation is correct and usable. Run
this BEFORE trusting nav to steer on these numbers.

No motors. No LEDs. Nothing moves. Camera only.

    python3 check_vision.py            live readout
    python3 check_vision.py --ruler    guided distance calibration

WHAT IT CHECKS
    1. Does Vision return anything at all
    2. Are the units what Navigation assumes (px and mm, not rad and m)
    3. Does distance match a tape measure - this is the one that decides
       whether "stops within 10 cm" passes or fails
    4. Which sign is left and which is right
    5. Is the loop fast enough (the brief wants >= 10 Hz)
    6. Does the maze-edge clearance track a real wall

Place this beside nav_hd_demo.py, or anywhere under System/.
"""

import math
import os
import statistics
import sys
import time

# find the sibling subsystem folders, same bootstrap nav uses
_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _name in sorted(os.listdir(_SYSTEM)):
    _path = os.path.join(_SYSTEM, _name)
    if os.path.isdir(_path) and not _name.startswith(("_", ".")):
        if _path not in sys.path:
            sys.path.insert(0, _path)

from vision_distance import VisionDistance, F_PX

AHEAD_PIXELS = 300      # must match nav_hd_demo.py


def to_nav_units(label_bearing_distance):
    """The exact conversion nav does. If this is wrong, nav is wrong."""
    label, bearing_px, distance_mm = label_bearing_distance
    return label, math.atan2(bearing_px, F_PX), distance_mm / 1000.0


def live():
    """Continuous readout in both Vision's units and Navigation's."""
    print("Vision -> Navigation interface check. Ctrl+C to stop.\n")
    print("Hold a yellow token in view and move it around. Watch that:")
    print("  - bearing goes NEGATIVE one side and POSITIVE the other")
    print("  - range falls smoothly as you bring it closer")
    print("  - clearance falls as the robot faces a wall\n")

    vision = VisionDistance()
    frame_times = []
    try:
        while True:
            t0 = time.monotonic()
            dets = vision.get_detections()
            frame_times.append(time.monotonic() - t0)
            frame_times = frame_times[-20:]
            hz = 1.0 / statistics.mean(frame_times) if frame_times else 0.0

            victims = [d for d in dets if d[0] == "Yellow" and d[2] > 0]
            edges = [d for d in dets if d[0] == "MazeEdge" and d[2] > 0
                     and abs(d[1]) < AHEAD_PIXELS]

            line = f"{hz:5.1f} Hz | "
            if victims:
                label, bearing_px, dist_mm = min(victims, key=lambda d: d[2])
                _, bearing_rad, range_m = to_nav_units((label, bearing_px, dist_mm))
                side = "LEFT " if bearing_px < 0 else "RIGHT"
                line += (f"victim {dist_mm:7.1f} mm = {range_m:5.3f} m  "
                         f"bearing {bearing_px:+5d} px = {math.degrees(bearing_rad):+6.1f} deg {side} | ")
            else:
                line += "no victim                                              | "

            if edges:
                line += f"clearance {min(d[2] for d in edges)/1000.0:5.3f} m"
            else:
                line += "clearance   none ahead"

            print(line)
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        vision.close()
        if hz < 10:
            print(f"\nWARNING: {hz:.1f} Hz is below the 10 Hz the brief asks for.")


def ruler():
    """Guided calibration. Measure, compare, get a correction factor.

    This is the single most important number in the whole system: if
    Vision's distance is wrong by a factor, the robot stops at the wrong
    place, and stopping distance is what the criterion grades."""
    print("Distance calibration\n")
    print("You need a tape measure and a yellow victim token.")
    print("Measure from the CAMERA LENS to the token each time.\n")

    vision = VisionDistance()
    results = []
    try:
        for target_mm in (150, 300, 500, 800):
            input(f"Place the token at {target_mm} mm, centred, then press Enter...")

            samples = []
            for _ in range(15):
                dets = vision.get_detections()
                victims = [d for d in dets if d[0] == "Yellow" and d[2] > 0]
                if victims:
                    samples.append(min(victims, key=lambda d: d[2])[2])
                time.sleep(0.1)

            if not samples:
                print("  no victim detected - check lighting and framing\n")
                continue

            reported = statistics.median(samples)
            spread = max(samples) - min(samples)
            ratio = reported / target_mm
            results.append((target_mm, reported, ratio))
            print(f"  actual {target_mm} mm -> reported {reported:7.1f} mm  "
                  f"(x{ratio:.2f}, spread {spread:.0f} mm)\n")

        if not results:
            print("No usable readings.")
            return

        print("=" * 58)
        ratios = [r for _, _, r in results]
        mean_ratio = statistics.mean(ratios)
        print(f"Average reported/actual = {mean_ratio:.3f}")

        if 0.9 <= mean_ratio <= 1.1:
            print("Distance is accurate. Nav's STOP_RANGE can be trusted.")
        else:
            print(f"Distance is out by a factor of {mean_ratio:.2f}.")
            print("Two ways to fix it:")
            print(f"  - Vision: divide the distance result by {mean_ratio:.2f}")
            print(f"  - Nav:    set STOP_RANGE = {0.08 * mean_ratio:.3f}")
            print("    (to physically stop at 8 cm given this reading)")

        if len(ratios) > 1 and (max(ratios) - min(ratios)) > 0.25:
            print("\nWARNING: the error changes with distance, so a single")
            print("correction factor will not fix it. The formula itself")
            print("needs work - likely CROP_Y or H_CAM.")
        print("=" * 58)

    except KeyboardInterrupt:
        pass
    finally:
        vision.close()


if __name__ == "__main__":
    if "--ruler" in sys.argv:
        ruler()
    else:
        live()
