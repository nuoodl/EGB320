"""
EGB320 Navigation - Milestone 2 HD demonstration
================================================
Navigation subsystem only. No vision code lives here - it comes from
vision_distance.py, which is Olive's pipeline. If she retunes the camera
or the distance maths, nothing in this file changes.

Targets one criterion, deliberately nothing more:

    "On approved embedded hardware, the navigation system uses live
     perception or sensor input to move from the Base Zone through a
     simplified maze to an exposed victim that is not initially visible
     from the starting pose, stops within 10 cm, and issues the Found
     Victim, green LED, and collection commands. Behaviour is accurate,
     repeatable, and robust to minor placement variation."

No cell mapping, no flood-fill, no return to base, no multi-victim logic.
Those are Milestone 3, and each one is another way to fail on demo day.

No dead reckoning anywhere. Every decision comes from the current frame.
That is the direct answer to "robust to minor placement variation" - the
robot never has a stored position that can be wrong, so starting it a few
centimetres off changes nothing.

WHAT THIS SUBSYSTEM RECEIVES
    vision_distance.VisionDistance.get_detections()
      -> [(label, bearing_px, distance_mm), ...]
         "Yellow"   the victim tokens
         "MazeEdge" maze structure, used here as forward clearance
         bearing_px  pixels right of image centre, negative = left
         distance_mm millimetres

    Those units are NOT what a control loop wants. Converting them to
    radians and metres is this file's job and happens in read_world().

WHAT THIS SUBSYSTEM SENDS
    velocity commands  -> mobility.drive(v, w)  (Olly's Mobility module)
    LED state          -> GPIO
    collection command -> RescueLink, over UART to the Nano running
                          Rescue Collection (see the protocol in that class)

RUN
    python3 nav_hd_demo.py            the demonstration
    python3 nav_hd_demo.py --check    hardware check, no driving
    python3 nav_hd_demo.py --look     print what it perceives, no driving

NOT TESTED ON HARDWARE. The CALIBRATE constants need work.

Team 4 - Navigation
"""

import math
import os
import sys
import time

# --- find the sibling subsystem folders -----------------------------------
# This file lives in System/Nagivation/, while the modules it needs are in
# System/Vision/, System/Mobility/ and System/motor_controller/. Python
# only searches this script's own folder, so without this every import
# below fails with ModuleNotFoundError.
#
# Every subfolder of System/ is added, so it does not matter which one
# controller.py ended up in, and moving files between them will not break
# it. Works no matter what directory you run the script from.
_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _name in sorted(os.listdir(_SYSTEM)):
    _path = os.path.join(_SYSTEM, _name)
    if os.path.isdir(_path) and not _name.startswith(("_", ".")):
        if _path not in sys.path:
            sys.path.insert(0, _path)
# --------------------------------------------------------------------------

import serial
from gpiozero import LED

import mobility
from vision_distance import VisionDistance, F_PX


# ---------------------------------------------------------------------------
# CALIBRATE
# ---------------------------------------------------------------------------
DRIVE_SPEED = 0.08       # m/s exploring
APPROACH_SPEED = 0.04    # m/s closing on the victim
TURN_SPEED = 0.5         # rad/s

WALL_AHEAD = 0.20        # m - maze structure closer than this blocks us
STOP_RANGE = 0.08        # m - stop here. Deliberately UNDER the 10 cm rule
                         # so vision error still lands inside it. Measure
                         # the real gap and adjust.
AIM_TOLERANCE = math.radians(8)
TURN_GAIN = 1.2          # rad/s per rad of bearing error. If the robot
                         # turns AWAY from the victim, negate this.

AHEAD_PIXELS = 300       # a MazeEdge blob within this many px of centre
                         # counts as being in front of us

CONTROL_HZ = 10          # the brief requires >= 10 Hz
TIME_LIMIT = 180         # s safety cap on a demo run

# Rescue Collection runs on an Arduino Nano.
#
# CONNECTION - USB cable, Nano to a Pi USB port. Recommended.
#   Nothing to wire, no level shifting (USB handles it), and the Nano can
#   still be reprogrammed without unplugging anything. Shows up as
#   /dev/ttyUSB0 on CH340-based Nanos or /dev/ttyACM0 on genuine ones -
#   the list below tries both so it does not matter which you have.
#
#   Power: do not feed the Nano's 5V pin from the regulator while USB is
#   also plugged in. Either let USB power it, or feed VIN (7-12V).
#
# ALTERNATIVE - Pi GPIO UART, if the USB port is needed elsewhere.
#   Pi TXD GPIO14/pin 8 -> Nano D0 (RX), and Nano D1 (TX) -> Pi RXD
#   GPIO15/pin 10 THROUGH A LEVEL SHIFTER OR 1k/2k DIVIDER. That is about
#   logic levels, not power: the Nano's TX swings to 5V and the Pi's GPIO
#   is 3.3V and not 5V tolerant, so a separate power supply does not make
#   it safe. Ground must be common (shared regulator output gives this).
#   Then add "/dev/serial0" to the list below, and enable the UART once:
#     sudo raspi-config > Interface Options > Serial Port
#       login shell NO, hardware YES, reboot
#   Note the GPIO UART blocks USB uploads to the Nano while D0/D1 are
#   connected - you have to unplug them to reprogram it.
RESCUE_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/serial0"]
RESCUE_BAUD = 115200           # must match the Nano sketch
RESCUE_TIMEOUT = 8.0           # s to wait for the mechanism to finish

GREEN_LED_PIN = 4        # BCM numbering. Green is the only LED the
                         # Milestone 2 criteria ask for - it must come ON
                         # in response to detection, so it stays off until
                         # the victim is found. Yellow/red are final-demo
                         # states and are not part of this milestone.


# ---------------------------------------------------------------------------
# Turning Vision's output into navigation quantities
# ---------------------------------------------------------------------------

def read_world(vision):
    """One frame -> (victim, clearance_m).

    victim is (range_m, bearing_rad) for the nearest yellow token, else
    None. clearance_m is the distance to the nearest maze structure
    roughly ahead.

    This is the only place Vision's pixels and millimetres get converted.
    Everything below works in radians and metres."""
    victim = None
    nearest_victim_mm = None
    ahead_mm = []

    for label, bearing_px, distance_mm in vision.get_detections():
        if distance_mm <= 0:
            continue

        if label == "Yellow":
            if nearest_victim_mm is None or distance_mm < nearest_victim_mm:
                nearest_victim_mm = distance_mm
                victim = (distance_mm / 1000.0,
                          math.atan2(bearing_px, F_PX))

        elif label == "MazeEdge" and abs(bearing_px) < AHEAD_PIXELS:
            ahead_mm.append(distance_mm)

    clearance_m = (min(ahead_mm) / 1000.0) if ahead_mm else 99.0
    return victim, clearance_m


# ---------------------------------------------------------------------------
# Hardware this subsystem drives
# ---------------------------------------------------------------------------

class Robot:
    def __init__(self):
        self.green = LED(GREEN_LED_PIN)

    def drive(self, v, w):
        """Hand a velocity command to Mobility. v forward m/s, w turn
        rad/s. How that becomes wheel speeds is their side of the
        interface, not ours - if they change motors, gearing or the
        controller, nothing in this file changes."""
        mobility.drive(v, w)

    def stop(self):
        mobility.stop()

    def green_on(self):
        """Found Victim indication. Graded on coming on IN RESPONSE to
        detection, so never light it before the victim is confirmed."""
        self.green.on()

    def green_off(self):
        self.green.off()

    def shutdown(self):
        self.stop()
        self.green.off()


class RescueLink:
    """The Navigation -> Rescue Collection interface, over UART to the Nano.

    PROTOCOL (agree this with whoever owns Rescue Collection - it is the
    contract between the two subsystems, and belongs on the architecture
    diagram):

        Pi  -> Nano   "COLLECT\n"    run the collection sequence
        Nano -> Pi    "OK <label>\n" collected, victim secured
                      "FAIL <why>\n" attempted and failed

        Pi  -> Nano   "PING\n"       are you alive
        Nano -> Pi    "PONG\n"

    Nothing here raises during a run. If the link drops mid-demo you want
    to know about it and carry on, not have the robot die holding a
    victim."""

    def __init__(self, ports=None, baud=RESCUE_BAUD):
        self.ser = None
        for port in (ports or RESCUE_PORTS):
            try:
                self.ser = serial.Serial(port, baud, timeout=0.5)
            except Exception:
                continue    # not this one, try the next
            # The Nano resets when the port opens - let it boot before
            # the first command, or the command gets swallowed.
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            print(f"[RESCUE] link open on {port}")
            return
        print(f"[RESCUE] no Nano found on any of {ports or RESCUE_PORTS}")

    def _ask(self, cmd, timeout):
        """Send a command, wait for one reply line. None on failure."""
        if self.ser is None:
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
        except Exception as exc:
            print(f"[RESCUE] write failed: {exc}")
            return None

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception as exc:
                print(f"[RESCUE] read failed: {exc}")
                return None
            if line:
                return line
        return None

    def ping(self):
        return self._ask("PING", 2.0) == "PONG"

    def collect(self):
        """Issue the collection command. Returns (success, label).

        The timeout is generous because this waits on a physical
        mechanism actuating, not just a message round trip."""
        reply = self._ask("COLLECT", RESCUE_TIMEOUT)

        if reply is None:
            print("[RESCUE] no reply - link down or mechanism did not respond")
            return False, "no reply"
        if reply.startswith("OK"):
            label = reply[2:].strip() or "victim"
            return True, label
        print(f"[RESCUE] {reply}")
        return False, reply

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Behaviour
# ---------------------------------------------------------------------------

def explore(robot, clearance):
    """Forward while clear, turn when blocked.

    Always turning the SAME way is deliberate: it makes runs repeatable,
    which the criterion grades. Alternating or random turns would make
    two runs from the same start diverge."""
    if clearance > WALL_AHEAD:
        robot.drive(DRIVE_SPEED, 0.0)
    else:
        robot.drive(0.0, TURN_SPEED)


def approach(robot, range_m, bearing_rad, clearance):
    """Close on the victim. True once inside STOP_RANGE.

    Turns and drives together rather than stop-turn-creep: smoother, and
    it keeps the victim in frame instead of swinging the camera off it."""
    if range_m <= STOP_RANGE:
        robot.stop()
        return True

    if abs(bearing_rad) > AIM_TOLERANCE:
        turn = max(-TURN_SPEED, min(TURN_SPEED, TURN_GAIN * bearing_rad))
        forward = APPROACH_SPEED * 0.4     # keep creeping while correcting
    else:
        turn = 0.0
        forward = APPROACH_SPEED

    # If the victim sits against maze structure, don't push into it -
    # hold position and let the range check decide.
    if clearance < WALL_AHEAD * 0.6:
        forward = 0.0

    robot.drive(forward, turn)
    return False


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def run():
    vision = VisionDistance()
    robot = Robot()
    rescue = RescueLink()
    started = time.monotonic()
    period = 1.0 / CONTROL_HZ
    state = "SEARCH"

    print("[BASE] leaving base zone, searching")
    robot.green_off()     # must not be lit before detection

    try:
        while time.monotonic() - started < TIME_LIMIT:
            tick = time.monotonic()
            victim, clearance = read_world(vision)

            if state == "SEARCH":
                if victim is not None:
                    r, b = victim
                    print(f"[DETECT] victim {r:.3f} m, {math.degrees(b):+.1f} deg")
                    state = "APPROACH"
                else:
                    explore(robot, clearance)

            elif state == "APPROACH":
                if victim is None:
                    # Lost it - keep turning slowly rather than giving up
                    # on a single missed frame.
                    robot.drive(0.0, TURN_SPEED * 0.5)
                else:
                    r, b = victim
                    if approach(robot, r, b, clearance):
                        # The three things the criterion asks for, in order.
                        print(f"[FOUND VICTIM] stopped at {r:.3f} m")
                        robot.green_on()
                        print("[COLLECT] commanding Rescue Collection")
                        ok, label = rescue.collect()
                        if ok:
                            print(f"[COLLECTED] {label}")
                            state = "DONE"
                        else:
                            print("[RETRY] collection failed, re-approaching")
                            robot.green_off()

            elif state == "DONE":
                robot.stop()
                print("[DONE] demonstration complete")
                return

            time.sleep(max(0.0, period - (time.monotonic() - tick)))

        print("[TIMEOUT] time limit reached")

    finally:
        robot.shutdown()
        vision.close()
        rescue.close()


def look():
    """Print what navigation perceives, in ITS units. No driving.
    Check these against a tape measure before trusting them to steer."""
    vision = VisionDistance()
    try:
        for _ in range(40):
            victim, clearance = read_world(vision)
            if victim:
                r, b = victim
                print(f"victim {r:6.3f} m  {math.degrees(b):+6.1f} deg   "
                      f"clearance {clearance:6.3f} m")
            else:
                print(f"no victim                       clearance {clearance:6.3f} m")
            time.sleep(0.2)
    finally:
        vision.close()


def check():
    """Hardware check, no driving. Run before every demo."""
    robot = Robot()
    try:
        print("Green LED:")
        robot.green_on()
        time.sleep(1.0)
        robot.green_off()

        print("Motors: brief nudge forward")
        robot.drive(DRIVE_SPEED, 0.0)
        time.sleep(0.5)
        robot.stop()

        print("Vision:")
        vision = VisionDistance()
        victim, clearance = read_world(vision)
        print(f"  victim={victim}  clearance={clearance:.3f} m")
        vision.close()

        print("Rescue Collection link:")
        rescue = RescueLink()
        print("  reachable" if rescue.ping() else
              "  NO RESPONSE - check wiring, divider, and the Nano sketch")
        rescue.close()

        print("\nCheck complete.")
    finally:
        robot.shutdown()


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--look" in sys.argv:
        look()
    else:
        run()