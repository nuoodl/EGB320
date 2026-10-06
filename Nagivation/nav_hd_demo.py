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

HOW IT SEARCHES - left-wall following
    The left ultrasonic sensor decides WHERE to go: when the left wall
    disappears (a side opening) the robot arcs left into it, and when
    vision sees a wall ahead it turns right on the spot. Following one
    wall like this reaches every branch of the maze, so a victim that is
    out of sight from the start is always found, and the robot takes the
    same route every run.

    Both side sensors decide how to drive STRAIGHT: with a wall on each
    side the robot centres between them, with only one it holds a set
    distance off that wall. Two walls give a much steadier line than one,
    because the robot's own weave cancels out of the difference.

No stored position. Every decision comes from the current readings; the
only timers are guards on how long a turn may last. Starting a few
centimetres off just means the wall controller pulls it back in.

WHAT THIS SUBSYSTEM RECEIVES
    vision_distance.VisionDistance.get_detections()
      -> [(label, bearing_px, distance_mm), ...]
         "Yellow"   the victim tokens
         "MazeEdge" maze structure, used here as forward clearance
         bearing_px  pixels right of image centre, negative = left
         distance_mm millimetres
    HC-SR04 ultrasonics on the left and right sides (gpiozero
    DistanceSensor), metres.

    Converting those to radians and metres is this file's job and happens
    in read_world(), Robot.left_m() and Robot.right_m().

WHAT THIS SUBSYSTEM SENDS
    velocity commands  -> mobility.drive(v, w)  (Olly's Mobility module)
    LED state          -> GPIO
    collection command -> RescueLink, over UART to the Nano running
                          Rescue Collection (see the protocol in that class)

WIRING (BCM numbers)
    Right ultrasonic   TRIG GPIO 23 (pin 16)   ECHO GPIO 24 (pin 18)
    Left ultrasonic    TRIG GPIO 17 (pin 11)   ECHO GPIO 27 (pin 13)
    Green LED          GPIO 4 (pin 7)
    Both ECHO lines go through a 1k/2k divider - the HC-SR04 echo is 5 V.

RUN
    python3 nav_hd_demo.py            the demonstration
    python3 nav_hd_demo.py --check    hardware check (short turn, no driving off)
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
_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _name in sorted(os.listdir(_SYSTEM)):
    _path = os.path.join(_SYSTEM, _name)
    if os.path.isdir(_path) and not _name.startswith(("_", ".")):
        if _path not in sys.path:
            sys.path.insert(0, _path)
# --------------------------------------------------------------------------

import serial
from gpiozero import LED, DistanceSensor

import mobility
from vision_distance import VisionDistance, F_PX


# ---------------------------------------------------------------------------
# CALIBRATE
# ---------------------------------------------------------------------------
W_LEFT = 1.0             # sign of w that turns the robot LEFT. Run --check:
                         # if the "turning LEFT" step turns right, set -1.0.
                         # Every turn in this file goes through this.

DRIVE_SPEED = 0.05       # m/s exploring. Mobility measured ~0.052 m/s max,
                         # so asking for more just gets clipped.
APPROACH_SPEED = 0.04    # m/s closing on the victim
TURN_SPEED = 0.5         # rad/s on-the-spot turns

WALL_AHEAD = 0.20        # m - maze structure closer than this blocks us
STOP_RANGE = 0.08        # m - stop here. Deliberately UNDER the 10 cm rule
                         # so vision error still lands inside it.
AIM_TOLERANCE = math.radians(8)
TURN_GAIN = 1.2          # rad/s per rad of bearing error

AHEAD_PIXELS = 300       # a MazeEdge blob within this many px of centre
                         # counts as being in front of us

# Side ultrasonics (HC-SR04). The echo pins output 5 V - they MUST go
# through a divider (1k/2k) into the Pi.
LEFT_TRIG_PIN = 17       # BCM
LEFT_ECHO_PIN = 27
RIGHT_TRIG_PIN = 23
RIGHT_ECHO_PIN = 24
SONIC_MAX = 1.0          # m - readings at this are "nothing there"

LEFT_TARGET = 0.08       # m - left reading with the robot centred in a
                         # 280 mm corridor. MEASURE: put it centred, run
                         # --look, copy the number here.
RIGHT_TARGET = 0.08      # m - same for the right sensor. The two can differ
                         # if the sensors aren't mounted symmetrically.
LEFT_OPEN = 0.25         # m - above this, the left wall has ended (opening)
RIGHT_OPEN = 0.25        # m - above this, no usable wall on the right

WALL_KP = 4.0            # rad/s per m off centre
WALL_KD = 1.0            # rad/s per (m/s) - damps the weave. 0 disables.
STEER_MAX = 0.4          # rad/s cap on wall steering

ARC_RADIUS = 0.14        # m - half a cell: arcs round the post into the
                         # middle of the side corridor
ARC_SPEED = 0.04         # m/s during the arc (w = ARC_SPEED / ARC_RADIUS)

CONFIRM = 2              # readings in a row before committing to a turn
LOST_TIME = 1.0          # s without seeing the victim before searching again
MAX_COLLECT_TRIES = 3

CONTROL_HZ = 10          # the brief requires >= 10 Hz
TIME_LIMIT = 180         # s safety cap on a demo run

# Rescue Collection runs on an Arduino Nano over USB serial.
# /dev/ttyUSB0 (CH340 Nanos) or /dev/ttyACM0 (genuine) - both are tried.
# /dev/serial0 is the GPIO UART fallback (needs a level shifter on Nano TX).
RESCUE_PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/serial0"]
RESCUE_BAUD = 115200           # must match the Nano sketch
RESCUE_TIMEOUT = 8.0           # s to wait for the mechanism to finish

GREEN_LED_PIN = 25        # BCM. Stays off until the victim is found.


# ---------------------------------------------------------------------------
# Turning Vision's output into navigation quantities
# ---------------------------------------------------------------------------

def read_world(vision):
    """One frame -> (victim, clearance_m).

    victim is (range_m, bearing_rad) for the nearest yellow token, else
    None. bearing_rad is positive to the RIGHT. clearance_m is the
    distance to the nearest maze structure roughly ahead."""
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
        # queue_len=5: gpiozero already returns the median of the last
        # few pings. Longer smooths more but lags - at 5 cm/s a long queue
        # means the robot is well past a corner before it notices.
        # The two sensors face opposite ways, so they rarely hear each
        # other. If readings jump around, crosstalk is the first suspect.
        self.left = DistanceSensor(echo=LEFT_ECHO_PIN, trigger=LEFT_TRIG_PIN,
                                   max_distance=SONIC_MAX, queue_len=5)
        self.right = DistanceSensor(echo=RIGHT_ECHO_PIN, trigger=RIGHT_TRIG_PIN,
                                    max_distance=SONIC_MAX, queue_len=5)

    @staticmethod
    def _read(sensor):
        d = sensor.distance
        return None if d >= SONIC_MAX - 0.01 else d

    def left_m(self):
        """Left wall distance in metres, or None if nothing in range."""
        return self._read(self.left)

    def right_m(self):
        """Right wall distance in metres, or None if nothing in range."""
        return self._read(self.right)

    def drive(self, v, w):
        """Hand a velocity command to Mobility. v forward m/s, w turn rad/s."""
        mobility.drive(v, w)

    def stop(self):
        mobility.stop()

    def green_on(self):
        self.green.on()

    def green_off(self):
        self.green.off()

    def shutdown(self):
        self.stop()
        self.green.off()
        self.left.close()
        self.right.close()


class RescueLink:
    """The Navigation -> Rescue Collection interface, over UART to the Nano.

        Pi  -> Nano   "COLLECT\\n"    run the collection sequence
        Nano -> Pi    "OK <label>\\n" collected, victim secured
                      "FAIL <why>\\n" attempted and failed

        Pi  -> Nano   "PING\\n"       are you alive
        Nano -> Pi    "PONG\\n"

    Nothing here raises during a run."""

    def __init__(self, ports=None, baud=RESCUE_BAUD):
        self.ser = None
        for port in (ports or RESCUE_PORTS):
            try:
                self.ser = serial.Serial(port, baud, timeout=0.5)
            except Exception:
                continue
            time.sleep(2.0)     # the Nano resets when the port opens
            self.ser.reset_input_buffer()
            print(f"[RESCUE] link open on {port}")
            return
        print(f"[RESCUE] no Nano found on any of {ports or RESCUE_PORTS}")

    def _ask(self, cmd, timeout):
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
        reply = self._ask("COLLECT", RESCUE_TIMEOUT)
        if reply is None:
            print("[RESCUE] no reply - link down or mechanism did not respond")
            return False, "no reply"
        if reply.startswith("OK"):
            return True, reply[2:].strip() or "victim"
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

def clamp(v, lim):
    return max(-lim, min(lim, v))


def side_error(left, right):
    """How far the robot is from where it should be, in metres, positive =
    too far RIGHT (so steer left). Also returns which walls it used.

    Both walls: centre between them. The robot's weave moves it closer to
    one and further from the other by the same amount, so the difference
    is a clean measure of position, and it doesn't depend on LEFT_TARGET /
    RIGHT_TARGET being calibrated exactly.
    One wall: hold the calibrated distance off that wall.
    Neither: None."""
    left_ok = left is not None and left <= LEFT_OPEN
    right_ok = right is not None and right <= RIGHT_OPEN
    if left_ok and right_ok:
        # Subtract each sensor's own "centred" reading first, so an
        # asymmetric mount doesn't pull the robot to one side.
        return ((left - LEFT_TARGET) - (right - RIGHT_TARGET)) / 2, "both"
    if left_ok:
        return left - LEFT_TARGET, "left"
    if right_ok:
        return RIGHT_TARGET - right, "right"
    return None, None


class WallFollower:
    """Left-hand wall following, one control tick at a time.

    FOLLOW  drive straight, centred between the walls (or off whichever
            wall is there)
    ARC     left wall ended: arc left round the post into the opening,
            until the left sensor finds the new corridor's wall
    RIGHT   wall ahead: turn right on the spot until the front is clear
            and there's a wall on the left again (corner or dead end)

    Entering ARC or RIGHT commits to at least ~60 deg of turning, so each
    needs CONFIRM readings in a row first - one noisy frame or one missed
    ultrasonic echo shouldn't swing the robot off a straight corridor."""

    def __init__(self):
        self.state = "FOLLOW"
        self.since = time.monotonic()
        self.last_err = None
        self.last_mode = None
        self.last_t = None
        self.empty_arcs = 0     # arcs in a row that found no wall
        self.near = 0           # frames in a row with a wall ahead
        self.open = 0           # readings in a row with the left wall gone

    def reset(self):
        self._go("FOLLOW", time.monotonic())
        self.near = self.open = 0

    def _go(self, state, now):
        if state != self.state:
            print(f"[WALL] {self.state} -> {state}")
        self.state, self.since = state, now
        self.last_err = None    # don't difference across a state change

    def step(self, robot, left, right, clearance):
        self.near = self.near + 1 if clearance <= WALL_AHEAD else 0
        left_open = left is None or left > LEFT_OPEN
        self.open = self.open + 1 if left_open else 0
        # A state change hands over to the new state in the same tick, so
        # every tick ends with a command from the state the robot is in.
        for _ in range(3):
            if self._tick(robot, left, right, left_open, clearance):
                return
        robot.stop()

    def _tick(self, robot, left, right, left_open, clearance):
        """Issue this state's command and return True, or change state
        and return False."""
        now = time.monotonic()
        t = now - self.since
        quarter_arc = (math.pi / 2) / (ARC_SPEED / ARC_RADIUS)
        quarter_turn = (math.pi / 2) / TURN_SPEED

        if self.state == "FOLLOW":
            if self.near >= CONFIRM:
                self._go("RIGHT", now)
                return False
            if self.open >= CONFIRM and self.empty_arcs < 2:
                self._go("ARC", now)
                return False
            if not left_open:
                self.empty_arcs = 0

            err, mode = side_error(left, right)
            if err is None:
                self.last_err = None
                robot.drive(DRIVE_SPEED, 0.0)       # no wall either side: straight
                return True

            # Switching between "both"/"left"/"right" changes the reference,
            # so the error jumps - don't treat that jump as a rate.
            rate = 0.0
            if self.last_err is not None and mode == self.last_mode and now > self.last_t:
                rate = (err - self.last_err) / (now - self.last_t)
            self.last_err, self.last_mode, self.last_t = err, mode, now
            w = clamp(WALL_KP * err + WALL_KD * rate, STEER_MAX)
            robot.drive(DRIVE_SPEED, W_LEFT * w)    # too far right -> turn left
            return True

        if self.state == "ARC":
            if self.near >= CONFIRM:
                self._go("RIGHT", now)
                return False
            if t > 0.6 * quarter_arc and not left_open:
                self.empty_arcs = 0
                self._go("FOLLOW", now)
                return False
            if t > 1.3 * quarter_arc:
                self.empty_arcs += 1
                self._go("FOLLOW", now)
                return False
            robot.drive(ARC_SPEED, W_LEFT * ARC_SPEED / ARC_RADIUS)
            return True

        # RIGHT. Exit needs clearly more room than the entry threshold
        # (hysteresis), or noise near WALL_AHEAD bounces it straight back.
        clear = clearance > WALL_AHEAD * 1.25
        if (t > 0.7 * quarter_turn and clear and not left_open) or t > 4 * quarter_turn:
            self.near = 0
            self._go("FOLLOW", now)
            return False
        robot.drive(0.0, -W_LEFT * TURN_SPEED)
        return True


def approach(robot, range_m, bearing_rad, clearance):
    """Close on the victim. True once inside STOP_RANGE."""
    if range_m <= STOP_RANGE:
        robot.stop()
        return True

    if abs(bearing_rad) > AIM_TOLERANCE:
        # bearing is + to the right; turning right is -W_LEFT
        turn = -W_LEFT * clamp(TURN_GAIN * bearing_rad, TURN_SPEED)
        forward = APPROACH_SPEED * 0.4
    else:
        turn = 0.0
        forward = APPROACH_SPEED

    # Hold back only for structure CLOSER than the victim. Victims sit
    # against a wall, so that wall is always just behind the victim -
    # stopping for it would park the robot short of STOP_RANGE forever.
    if clearance < range_m - 0.03:
        forward = 0.0

    robot.drive(forward, turn)
    return False


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------

def fmt(v):
    return "  -- " if v is None else f"{v:5.3f}"


def run():
    vision = VisionDistance()
    robot = Robot()
    rescue = RescueLink()
    follower = WallFollower()
    started = time.monotonic()
    period = 1.0 / CONTROL_HZ
    state = "SEARCH"
    lost_since = None
    tries = 0

    print("[BASE] leaving base zone, searching")
    robot.green_off()

    try:
        while time.monotonic() - started < TIME_LIMIT:
            tick = time.monotonic()
            victim, clearance = read_world(vision)
            left = robot.left_m()
            right = robot.right_m()

            if state == "SEARCH":
                if victim is not None:
                    r, b = victim
                    print(f"[DETECT] victim {r:.3f} m, {math.degrees(b):+.1f} deg")
                    state = "APPROACH"
                    lost_since = None
                else:
                    follower.step(robot, left, right, clearance)

            elif state == "APPROACH":
                if victim is None:
                    # Hold still for a moment - spinning would swing the
                    # camera away from where it just was. Still gone: it
                    # was a false detection or it's hidden; search again.
                    robot.stop()
                    if lost_since is None:
                        lost_since = time.monotonic()
                    elif time.monotonic() - lost_since > LOST_TIME:
                        print("[LOST] victim out of view - searching again")
                        follower.reset()
                        state = "SEARCH"
                else:
                    lost_since = None
                    r, b = victim
                    if approach(robot, r, b, clearance):
                        print(f"[FOUND VICTIM] stopped at {r:.3f} m")
                        robot.green_on()
                        print("[COLLECT] commanding Rescue Collection")
                        ok, label = rescue.collect()
                        tries += 1
                        if ok:
                            print(f"[COLLECTED] {label}")
                            state = "DONE"
                        elif tries >= MAX_COLLECT_TRIES:
                            print(f"[COLLECT] failed {tries} times - stopping here")
                            state = "DONE"
                        else:
                            print("[RETRY] collection failed, trying again")

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
    Put the robot centred in a corridor and copy the left/right numbers
    into LEFT_TARGET / RIGHT_TARGET."""
    vision = VisionDistance()
    robot = Robot()
    try:
        for _ in range(40):
            victim, clearance = read_world(vision)
            left, right = robot.left_m(), robot.right_m()
            err, mode = side_error(left, right)
            v = (f"victim {victim[0]:6.3f} m  {math.degrees(victim[1]):+6.1f} deg"
                 if victim else "no victim                  ")
            e = "  --  " if err is None else f"{err * 100:+5.1f} cm ({mode})"
            print(f"{v}   clearance {clearance:6.3f} m   "
                  f"left {fmt(left)} m   right {fmt(right)} m   off-centre {e}")
            time.sleep(0.2)
    finally:
        robot.shutdown()
        vision.close()


def check():
    """Hardware check. Run before every demo."""
    robot = Robot()
    try:
        print("Green LED:")
        robot.green_on()
        time.sleep(1.0)
        robot.green_off()

        print("Side ultrasonics (5 readings) - cover one at a time to check")
        print("left and right aren't swapped:")
        for _ in range(5):
            print(f"  left {fmt(robot.left_m())} m   right {fmt(robot.right_m())} m")
            time.sleep(0.2)

        print("Motors: brief nudge forward")
        robot.drive(DRIVE_SPEED, 0.0)
        time.sleep(0.5)
        robot.stop()

        print("Motors: turning LEFT for 1 s - if it turns right, set W_LEFT = -1.0")
        robot.drive(0.0, W_LEFT * TURN_SPEED)
        time.sleep(1.0)
        robot.stop()

        print("Vision:")
        vision = VisionDistance()
        victim, clearance = read_world(vision)
        print(f"  victim={victim}  clearance={clearance:.3f} m")
        vision.close()

        print("Rescue Collection link:")
        rescue = RescueLink()
        print("  reachable" if rescue.ping() else
              "  NO RESPONSE - check the cable and the Nano sketch")
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