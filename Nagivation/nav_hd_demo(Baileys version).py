"""
EGB320 Navigation - Milestone 3 integrated rescue cycle (real robot)
===================================================================
Start in the base zone, explore the maze, find a victim, stop within 10 cm,
trigger collection, and bring the victim back to base.

HOW IT WORKS - grid mapping
    The maze is a grid of 280 mm cells. The robot moves exactly one cell at
    a time and keeps a map of every cell it has been in:
      - side walls from the left/right ultrasonics,
      - the wall ahead from the front ultrasonic,
      - its position from the wheel encoders (Mobility's read_encoders()).
    Exploration always heads for the nearest unexplored opening (breadth-
    first search over the map), so it remembers dead ends and blocked
    routes and never re-explores them. When the camera sees a victim it
    drives to it, stops within 10 cm, lights the green LED and commands
    Rescue Collection. It then backs out to the cell it came from and plans
    the shortest route home through the map.

    The map is printed as a drawing after every new cell, and again with
    the planned route home - the evidence that the robot remembers explored
    areas, dead ends, the victim location and the path back.

CLAMP SEQUENCE (ESP32 over UART)
    exploring, no victim in view    clamp idle
    victim detected                 LOWER   (clamp to 95 deg, open)
    victim lost close up            drive LOST_CREEP blind on the encoders
    (or reaches STOP_RANGE)         COLLECT (clamp to 40 deg)
    victim lost far away            RAISE, back to exploring
    in the base zone                RELEASE (clamp to 95 deg)

WHAT THIS SUBSYSTEM RECEIVES
    vision_distance.VisionDistance.get_detections()
        [(label, bearing_px, distance_mm), ...]  "Yellow" = victim,
        "MazeEdge" = maze structure (forward clearance). bearing_px is
        pixels right of centre.
    HC-SR04 ultrasonics, left and right (gpiozero DistanceSensor), metres.
    Wheel encoder ticks: mobility.read_encoders() -> (left, right), ticks
        since the last call. Mobility's control loop is the only thing that
        reads the board, so these are never stolen by it.

WHAT THIS SUBSYSTEM SENDS
    velocity commands  -> mobility.drive(v, w)
    LED state          -> GPIO
    LOWER / RAISE / COLLECT / RELEASE -> RescueLink, UART to the ESP32
        (protocol below). Pi TX GPIO14 -> ESP32 RX GPIO3, Pi RX GPIO15 <-
        ESP32 TX GPIO1, common GND.

RUN
    python3 nav_hd_demo.py              the demonstration
    python3 nav_hd_demo.py --check      hardware check: LED, sensors, encoders, motors, link
    python3 nav_hd_demo.py --look       live readings, no driving
    python3 nav_hd_demo.py --ticks      calibrate TICKS_PER_M
    python3 nav_hd_demo.py --spin       calibrate TRACK (encoder turning)

CALIBRATE BEFORE THE DEMO (in this order)
    1. --check   : W_LEFT, and that both encoders count UP going forward
    2. --ticks   : checks Mobility's TICKS_PER_MM against a tape measure
    3. --spin    : checks the wheelbase gives exact turns (TRACK)
    4. --look    : LEFT_TARGET, RIGHT_TARGET (robot centred in a corridor)
                   FRONT_CENTRED_CM (robot centred in a cell, facing a wall)

    Distance and wheelbase come from mobility.py (TICKS_PER_MM, WHEELBASE)
    so Mobility and Navigation always use the same numbers.

Team 4 - Navigation
"""

import math
import os
import sys
import time
from collections import deque

# --- find the sibling subsystem folders (System/Vision, System/Mobility ...) -
_SYSTEM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _name in sorted(os.listdir(_SYSTEM)):
    _path = os.path.join(_SYSTEM, _name)
    if os.path.isdir(_path) and not _name.startswith(("_", ".")):
        if _path not in sys.path:
            sys.path.insert(0, _path)
# ---------------------------------------------------------------------------

from gpiozero import LED, DistanceSensor

import mobility
from vision_distance import VisionDistance, F_PX


# ---------------------------------------------------------------------------
# CALIBRATE
# ---------------------------------------------------------------------------
W_LEFT = 1.0             # mobility.drive(): positive w = turn LEFT. Leave at 1.0 -
LEFT_TICK_SIGN = 1       # Mobility's closed loop needs the motors and encoders to
RIGHT_TICK_SIGN = 1      # agree, so if --check fails the fix is in mobility.py, not here
TICKS_PER_M = mobility.TICKS_PER_MM * 1000.0   # 1400/93 ticks per mm, from Mobility
TRACK = mobility.WHEELBASE                     # 0.119 m, from Mobility. If --spin
                                               # shows turns are off, set
                                               # TRACK = mobility.WHEELBASE * factor

LEFT_TARGET_CM = 8.0     # cm, left ultrasonic with the robot centred (--look)
RIGHT_TARGET_CM = 8.0    # cm, right ultrasonic with the robot centred
FRONT_CENTRED_CM = 6.0   # cm, FRONT ultrasonic with the robot centred in a
                         # cell facing the wall at the cell's edge (--look)

CRUISE_SPEED = 0.10      # m/s for most of each cell
DRIVE_SPEED = 0.05       # m/s for the last SLOW_ZONE of a cell, or with a wall
                         # coming up ahead - keeps stops precise between camera frames
SLOW_ZONE = 0.08         # m
APPROACH_SPEED = 0.04    # m/s closing on the victim
TURN_SPEED = 1.0         # rad/s -> each wheel 60 mm/s
TURN_SLOW = 0.3          # rad/s for the last TURN_SLOW_ZONE of a turn
TURN_SLOW_ZONE = math.radians(30)
TURN_TOL = math.radians(3)

# ---------------------------------------------------------------------------
# Maze and behaviour
# ---------------------------------------------------------------------------
CELL = 0.28              # m
SIDE_WALL_CM = 16.0      # cm, a side reading below this is a wall (centred ~8,
                         # off-centre up to ~12; an opening reads 20+)
FRONT_CENTRED = FRONT_CENTRED_CM / 100
FRONT_WALL = FRONT_CENTRED + 0.14   # front reading below this at a cell centre = wall ahead
                                    # (halfway between a wall at this cell's edge and the next)
FRONT_STOP = FRONT_CENTRED          # stop a move when the wall ahead is this close
FRONT_TOUCH = 0.03                  # m, front reading below this = about to touch something
SONIC_MAX_CM = 400.0     # cm, sensor range - matches the standalone HC-SR04 test that works.
                         # Readings at this are "nothing there"

WALL_KP = 8.0            # rad/s per m off centre
HEAD_KP = 1.5            # rad/s per rad off the corridor heading - also damps the
                         # wall steering, so the robot doesn't weave
HEAD_GUARD = math.radians(12)   # never let the walls steer it further off the
                                # corridor heading than this
STEER_MAX = 0.4          # rad/s
STALL_TIME = 2.0         # s with no encoder progress = blocked / stuck
TOUCH_CM = 2.5           # cm, a side reading below this = touching the wall
REANCHOR_GAIN = 0.4      # how fast wall evidence corrects the encoder heading
SIDE_JUMP_CM = 1.5       # cm, a side reading jumping this much between samples is a
                         # different surface (wall end, post), not the robot turning
# Ultrasonic settings above are in cm; the maths below works in metres.
LEFT_TARGET, RIGHT_TARGET = LEFT_TARGET_CM / 100, RIGHT_TARGET_CM / 100
SIDE_WALL, SONIC_MAX = SIDE_WALL_CM / 100, SONIC_MAX_CM / 100
TOUCH, SIDE_JUMP = TOUCH_CM / 100, SIDE_JUMP_CM / 100
LOST_CLOSE = 0.25        # m, a victim lost from view closer than this has gone under
                         # the camera - drive the rest of the way on the encoders
LOST_CREEP = 0.15        # m, how far to drive (encoders) once the victim drops out of
                         # view up close, before the clamp closes. Set to None to
                         # drive (last seen range - STOP_RANGE) instead.

SEEK_RANGE = 1.0         # m, chase a victim seen closer than this...
SEEK_BEARING = math.radians(30)     # ...and roughly ahead
STOP_RANGE = 0.08        # m, under the 10 cm rule so vision error still lands inside
AIM_TOLERANCE = math.radians(8)
TURN_GAIN = 1.2          # rad/s per rad of bearing error while approaching
AHEAD_PIXELS = 300       # MazeEdge within this many px of centre = ahead
LOST_TIME = 1.0          # s without seeing the victim before giving up the approach
APPROACH_TIMEOUT = 40.0  # s, give up an approach that takes longer than this
FRONT_CONFIRM = 2        # camera frames in a row before a "wall ahead" stops a move
PEEK_SIDES = False       # at a new junction, turn to look down unexplored side openings
                         # for a victim before carrying on (finds side victims sooner)
MAX_COLLECT_TRIES = 3

TIME_LIMIT = 420         # s, whole run
HOME_MARGIN = 30         # s spare on top of the estimated trip home
CELL_TIME = 12.0         # s per cell (move + turn + mapping) until it has measured its own
CONTROL_HZ = 10

# Hardware
LEFT_TRIG_PIN, LEFT_ECHO_PIN = 17, 27      # BCM. Echo lines via 1k/2k divider.
RIGHT_TRIG_PIN, RIGHT_ECHO_PIN = 23, 24
FRONT_TRIG_PIN, FRONT_ECHO_PIN = 5, 6      # pins 29 / 31
LED_PINS = {"green": 25}                    # add "red"/"yellow" here when wired
USE_RESCUE = True        # the ESP32 clamp is fitted. False: the robot still finds,
                         # approaches and stops at the victim and lights the green
                         # LED, but sends no clamp commands.
# The ESP32 is on the Pi's GPIO UART (GPIO14/15). The USB ports are fallbacks.
RESCUE_PORTS = ["/dev/serial0", "/dev/ttyUSB0", "/dev/ttyACM0"]
RESCUE_BAUD = 115200     # must match the ESP32 sketch
RESCUE_TIMEOUT = 8.0     # s to wait for the clamp to finish

# Grid directions. The robot starts in base cell (0, 0) facing "E" - out
# of the base. These are the robot's own map directions, not compass ones.
DIRS = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
ORDER = ["E", "N", "W", "S"]                # anticlockwise
OPPOSITE = {"E": "W", "W": "E", "N": "S", "S": "N"}
BASE = (0, 0)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lim):
    return max(-lim, min(lim, v))


def left_of(d):
    return ORDER[(ORDER.index(d) + 1) % 4]


def right_of(d):
    return ORDER[(ORDER.index(d) - 1) % 4]


def step(cell, d):
    dx, dy = DIRS[d]
    return (cell[0] + dx, cell[1] + dy)


def fmt(v):
    """An ultrasonic reading (metres inside the code) printed in cm."""
    return "  --  " if v is None else f"{v * 100:5.1f}cm"


# ===========================================================================
# Hardware: everything this subsystem reads and drives
# ===========================================================================

class Hardware:
    def __init__(self):
        try:
            self.vision = VisionDistance()
        except RuntimeError as exc:
            print("\n[CAMERA] couldn't open the camera - almost always another program has it.\n"
                  "  Check:   pgrep -af python\n"
                  "  Stop it: pkill -f nav_hd_demo.py ; pkill -f vision_distance.py\n"
                  "  Still failing? rpicam-hello --list-cameras - if no camera is listed,\n"
                  "  reseat the ribbon cable (power off) and reboot.\n")
            raise SystemExit(1) from exc
        self.left = DistanceSensor(echo=LEFT_ECHO_PIN, trigger=LEFT_TRIG_PIN,
                                   max_distance=SONIC_MAX, queue_len=5)
        # Front: shorter queue = less lag, it decides when a move stops.
        self.front = DistanceSensor(echo=FRONT_ECHO_PIN, trigger=FRONT_TRIG_PIN,
                                    max_distance=SONIC_MAX, queue_len=3)
        self.right = DistanceSensor(echo=RIGHT_ECHO_PIN, trigger=RIGHT_TRIG_PIN,
                                    max_distance=SONIC_MAX, queue_len=5)
        self.leds = {name: LED(pin) for name, pin in LED_PINS.items()}
        self.rescue = RescueLink() if USE_RESCUE else NoRescue()

    @staticmethod
    def _sonic(sensor):
        d = sensor.distance
        return None if d >= SONIC_MAX - 0.01 else d

    def left_m(self):
        return self._sonic(self.left)

    def front_m(self):
        """Front wall distance in metres, or 99.0 if nothing in range."""
        d = self._sonic(self.front)
        return 99.0 if d is None else d

    def right_m(self):
        return self._sonic(self.right)

    def look(self, draw=False):
        """One camera frame -> (victim, clearance_m). victim is (range_m,
        bearing_rad) with bearing POSITIVE TO THE RIGHT, or None.
        draw=True also has Vision draw its boxes into self.vision.last_frame."""
        victim, nearest, ahead = None, None, []
        for label, bearing_px, distance_mm in self.vision.get_detections(draw_on=True if draw else None):
            if distance_mm <= 0:
                continue
            if label == "Yellow":
                if nearest is None or distance_mm < nearest:
                    nearest = distance_mm
                    victim = (distance_mm / 1000.0, math.atan2(bearing_px, F_PX))
            elif label == "MazeEdge" and abs(bearing_px) < AHEAD_PIXELS:
                ahead.append(distance_mm)
        return victim, (min(ahead) / 1000.0 if ahead else 99.0)

    def encoders(self):
        """(left, right) ticks since the last call."""
        return mobility.read_encoders()

    def drive(self, v, w):
        mobility.drive(v, w)

    def stop(self):
        mobility.stop()

    def led(self, name, on=True):
        if name in self.leds:
            self.leds[name].on() if on else self.leds[name].off()

    def lower_clamp(self):
        """Victim in sight: clamp down to the open position (no reply)."""
        self.rescue.lower()

    def raise_clamp(self):
        """No victim: clamp back to idle (no reply)."""
        self.rescue.raise_clamp()

    def collect(self):
        return self.rescue.collect()

    def release(self):
        return self.rescue.release()

    def shutdown(self):
        self.stop()
        for led in self.leds.values():
            led.off()
        for sensor in (self.left, self.right, self.front):
            # gpiozero closes the trigger pin BEFORE stopping its ping thread,
            # so a ping mid-flight hits a missing pin and prints a traceback.
            # Stop the thread first.
            try:
                sensor._queue.stop()
            except Exception:
                pass
            sensor.close()
        self.vision.close()
        self.rescue.close()


class NoRescue:
    """Stands in when USE_RESCUE is False: nothing is sent."""

    def ping(self):
        return False

    def lower(self):
        pass

    def raise_clamp(self):
        pass

    def collect(self):
        return False, "no rescue hardware"

    def release(self):
        return False

    def close(self):
        pass


class RescueLink:
    """Navigation -> Rescue Collection, over UART to the ESP32 clamp controller.

        Pi -> ESP32  "LOWER\\n"    clamp to 95 deg (open). No reply.
        Pi -> ESP32  "RAISE\\n"    clamp back to idle. No reply.
        Pi -> ESP32  "COLLECT\\n"  open, then clamp to 40 deg
        ESP32 -> Pi  "OK victim\\n" or "FAIL <why>\\n"
        Pi -> ESP32  "RELEASE\\n"  open the clamp in the base zone
        ESP32 -> Pi  "OK released\\n" or "FAIL <why>\\n"
        Pi -> ESP32  "PING\\n"  ->  "PONG\\n"

    LOWER and RAISE are fire-and-forget so the control loop never stalls
    while the servo sweeps. Nothing here raises during a run."""

    def __init__(self, ports=None, baud=RESCUE_BAUD):
        import serial
        self.ser = None
        for port in (ports or RESCUE_PORTS):
            try:
                self.ser = serial.Serial(port, baud, timeout=0.5)
            except Exception:
                continue
            time.sleep(2.0)     # a USB-connected ESP32 resets when the port opens
            self.ser.reset_input_buffer()
            print(f"[RESCUE] link open on {port}")
            return
        print(f"[RESCUE] no ESP32 found on any of {ports or RESCUE_PORTS}")

    def send(self, cmd):
        """Fire and forget."""
        if self.ser is None:
            return
        try:
            self.ser.write((cmd + "\n").encode())
            self.ser.flush()
        except Exception as exc:
            print(f"[RESCUE] write failed: {exc}")

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

    def lower(self):
        self.send("LOWER")

    def raise_clamp(self):
        self.send("RAISE")

    def collect(self):
        reply = self._ask("COLLECT", RESCUE_TIMEOUT)
        if reply is None:
            print("[RESCUE] no reply to COLLECT")
            return False, "no reply"
        if reply.startswith("OK"):
            return True, reply[2:].strip() or "victim"
        print(f"[RESCUE] {reply}")
        return False, reply

    def release(self):
        reply = self._ask("RELEASE", RESCUE_TIMEOUT)
        if reply is None:
            print("[RESCUE] no reply to RELEASE")
            return False
        if not reply.startswith("OK"):
            print(f"[RESCUE] {reply}")
        return reply.startswith("OK")

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


# ===========================================================================
# Odometry from the wheel encoders
# ===========================================================================

class Odometry:
    def __init__(self, hw):
        self.hw = hw
        self.x = self.y = self.th = 0.0
        hw.encoders()               # clear any stale count

    def update(self):
        dl, dr = self.hw.encoders()
        sl = LEFT_TICK_SIGN * dl / TICKS_PER_M
        sr = RIGHT_TICK_SIGN * dr / TICKS_PER_M
        ds, dth = (sl + sr) / 2.0, (sr - sl) / TRACK
        mid = self.th + dth / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)


# ===========================================================================
# Driver: turns and one-cell moves
# ===========================================================================

def side_error(left, right):
    """Metres off where the robot should be, positive = too far RIGHT."""
    left_ok = left is not None and left < SIDE_WALL
    right_ok = right is not None and right < SIDE_WALL
    if left_ok and right_ok:
        return ((left - LEFT_TARGET) - (right - RIGHT_TARGET)) / 2, "both"
    if left_ok:
        return left - LEFT_TARGET, "left"
    if right_ok:
        return RIGHT_TARGET - right, "right"
    return None, None


class Driver:
    def __init__(self, hw):
        self.hw = hw
        self.odo = Odometry(hw)
        self.facing = "E"
        self.offset = 0.0           # odometry heading of map direction "E"

    def heading_of(self, d):
        return wrap(self.offset + ORDER.index(d) * math.pi / 2)

    def pose(self):
        self.odo.update()
        return self.odo.x, self.odo.y, self.odo.th

    def turn_to(self, target, timeout=12.0):
        """Turn on the spot to an odometry heading. True when there."""
        period = 1.0 / 20
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, _, th = self.pose()
            err = wrap(target - th)
            if abs(err) < TURN_TOL:
                self.hw.stop()
                return True
            rate = TURN_SPEED if abs(err) > TURN_SLOW_ZONE else TURN_SLOW
            self.hw.drive(0.0, W_LEFT * math.copysign(rate, err))
            time.sleep(period)
        self.hw.stop()
        print("[TURN] timed out - wheels not turning (battery?) or motors/encoders disagree")
        return False

    def face(self, d):
        self.turn_to(self.heading_of(d))
        self.facing = d

    def front_clearance(self, samples=3):
        """Median of a few front ultrasonic readings (metres, 99 = nothing)."""
        vals = []
        for _ in range(samples):
            vals.append(self.hw.front_m())
            time.sleep(0.05)
        vals.sort()
        return vals[len(vals) // 2]

    def forward_cell(self):
        """Drive one cell. True if it got there; on failure it reverses to
        where it started, so the map position stays right."""
        x0, y0, _ = self.pose()
        ref = self.heading_of(self.facing)
        period = 1.0 / CONTROL_HZ
        best, progress_t = 0.0, time.monotonic()
        touch_t = None
        close_frames = 0
        history = []                # (distance, off-centre, odometry angle, wall mode)
        ok = False
        while True:
            tick = time.monotonic()
            x, y, th = self.pose()
            dist = math.hypot(x - x0, y - y0)
            if dist >= CELL - 0.01:
                # (No camera "did the wall ahead get closer" check here: down a
                # long corridor the side walls' bottom edges sit in the camera's
                # "ahead" zone at a constant ~35 cm, which looked like being
                # pinned. Being stuck is caught by the encoder stall check below.)
                ok = True
                break
            clearance = self.hw.front_m()
            # one bad reading must not end the move - two failed moves mark a
            # wall on the map for good
            close_frames = close_frames + 1 if clearance < FRONT_STOP else 0
            if close_frames >= FRONT_CONFIRM:
                ok = dist > CELL / 2        # wall ahead: arrived if most of the way
                if not ok:
                    print(f"[DRIVE] something {clearance * 100:.0f} cm ahead after only "
                          f"{dist * 100:.0f} cm - not moving into it")
                break
            if dist > best + 0.005:
                best, progress_t = dist, tick
            elif tick - progress_t > STALL_TIME:
                print("[DRIVE] no progress - blocked")
                break

            left, right = self.hw.left_m(), self.hw.right_m()
            if any(v is not None and v < TOUCH for v in (left, right)):
                touch_t = touch_t or tick
                if tick - touch_t > 1.0:
                    print("[DRIVE] rubbing a wall - stopping")
                    break
            else:
                touch_t = None
            err, mode = side_error(left, right)
            rel = wrap(th - ref)                           # >0: turned left of the corridor
            # Re-anchor the encoder heading from the walls: how fast the
            # side offset changes with distance is the robot's real angle to
            # the corridor. Any difference from what the encoders say is
            # encoder drift - left alone it builds up over turns until the
            # robot drives into a wall.
            if err is not None:
                if history and (history[-1][3] != mode or abs(err - history[-1][1]) > SIDE_JUMP):
                    history.clear()     # different wall/surface - don't read a jump as a turn
                history.append((dist, err, rel, mode))
                back = next((h for h in reversed(history) if h[0] <= dist - 0.08), None)
                if back is not None:
                    true_rel = -(err - back[1]) / (dist - back[0])
                    drift = (rel + back[2]) / 2 - true_rel
                    if abs(drift) < math.radians(30):
                        self.offset = wrap(self.offset + REANCHOR_GAIN * 0.1 * drift)
                        ref = self.heading_of(self.facing)
                        rel = wrap(th - ref)
            else:
                history.clear()
            if err is None or abs(rel) > HEAD_GUARD:
                w = -HEAD_KP * rel                         # no walls / too far off: hold heading
            else:
                w = WALL_KP * err - HEAD_KP * rel          # too far right -> turn left
            near_end = CELL - dist < SLOW_ZONE or clearance < FRONT_STOP + 0.12
            speed = DRIVE_SPEED if near_end else CRUISE_SPEED
            self.hw.drive(speed, W_LEFT * clamp(w, STEER_MAX))
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
        self.hw.stop()
        if not ok:
            self.back_to(x0, y0)
        return ok

    def square_up_on_walls(self):
        """With walls on both sides, left + right is smallest when the robot
        is exactly square to them. Sweep +-15 deg, find that minimum, and
        reset the encoder heading to it. Uses the walls, not the encoders,
        so it fixes heading errors the encoders can't see."""
        base = self.heading_of(self.facing)
        best = None
        for deg in range(-15, 16, 5):
            self.turn_to(wrap(base + math.radians(deg)))
            left, right = self.hw.left_m(), self.hw.right_m()
            if left is None or right is None or left > SIDE_WALL * 1.5 or right > SIDE_WALL * 1.5:
                continue
            total = left + right
            if best is None or total < best[0]:
                best = (total, self.pose()[2])
        if best is None:
            self.turn_to(base)
            return False
        self.offset = wrap(self.offset + wrap(best[1] - base))
        self.face(self.facing)
        print(f"[ALIGN] squared up on the walls (heading was "
              f"{math.degrees(wrap(best[1] - base)):+.0f} deg off)")
        return True

    def recentre(self):
        """Shift sideways onto the corridor centre line: turn 30 deg toward
        it, drive, turn back, reverse to the same spot along the corridor."""
        err, _ = side_error(self.hw.left_m(), self.hw.right_m())
        if err is None or abs(err) < 0.02:
            return
        err = clamp(err, 0.06)
        print(f"[ALIGN] {err * 100:+.1f} cm off centre - shifting back")
        base = self.heading_of(self.facing)
        ang = math.radians(30) * (1 if err > 0 else -1)   # too far right -> angle left
        self.turn_to(wrap(base + ang))
        x0, y0, _ = self.pose()
        self.creep(abs(err) / math.sin(math.radians(30)))
        self.turn_to(base)
        x1, y1, _ = self.pose()
        self.back_by(math.hypot(x1 - x0, y1 - y0) * math.cos(math.radians(30)))

    def back_by(self, dist):
        x0, y0, _ = self.pose()
        _, _, hold = self.pose()
        deadline = time.monotonic() + dist / DRIVE_SPEED + 3.0
        while time.monotonic() < deadline:
            x, y, th = self.pose()
            if math.hypot(x - x0, y - y0) >= dist:
                break
            self.hw.drive(-DRIVE_SPEED, W_LEFT * clamp(HEAD_KP * wrap(hold - th), STEER_MAX))
            time.sleep(1.0 / 20)
        self.hw.stop()

    def back_to(self, x0, y0, timeout=15.0):
        """Reverse in a straight line to a recorded point."""
        _, _, hold = self.pose()
        deadline = time.monotonic() + timeout
        closest = None
        while time.monotonic() < deadline:
            x, y, th = self.pose()
            d = math.hypot(x - x0, y - y0)
            if d < 0.01 or (closest is not None and d > closest + 0.005):
                break
            closest = d if closest is None else min(closest, d)
            self.hw.drive(-DRIVE_SPEED, W_LEFT * clamp(HEAD_KP * wrap(hold - th), STEER_MAX))
            time.sleep(1.0 / 20)
        self.hw.stop()

    def creep(self, dist):
        """Drive straight ahead a short distance on the encoders."""
        if dist <= 0:
            return
        x0, y0, hold = self.pose()
        deadline = time.monotonic() + dist / APPROACH_SPEED + 3.0
        while time.monotonic() < deadline:
            x, y, th = self.pose()
            if math.hypot(x - x0, y - y0) >= dist:
                break
            self.hw.drive(APPROACH_SPEED, W_LEFT * clamp(HEAD_KP * wrap(hold - th), STEER_MAX))
            time.sleep(1.0 / 20)
        self.hw.stop()

    def approach(self):
        """Drive to the victim in view. True once inside STOP_RANGE."""
        period = 1.0 / CONTROL_HZ
        lost_since = None
        last_r = None
        started = time.monotonic()
        x0, y0, _ = self.pose()
        best, progress_t = 0.0, started
        while True:
            tick = time.monotonic()
            x, y, _ = self.pose()
            if tick - started > APPROACH_TIMEOUT:
                self.hw.stop()
                print("[APPROACH] taking too long - giving up")
                return False
            moved = math.hypot(x - x0, y - y0)
            if moved > best + 0.005:
                best, progress_t = moved, tick
            elif tick - progress_t > STALL_TIME:
                # Driving at it but not moving: pressed against the victim
                # while Vision still reads a little over STOP_RANGE.
                self.hw.stop()
                print("[FOUND VICTIM] can't get any closer - stopped at it")
                return True
            victim, _ = self.hw.look()
            if self.hw.front_m() < FRONT_TOUCH:
                self.hw.stop()
                print("[FOUND VICTIM] front sensor: right up against it - stopped")
                return True
            if victim is None:
                self.hw.stop()
                if last_r is not None and last_r < LOST_CLOSE:
                    # Close up, the token drops out of the camera's view.
                    # It's straight ahead - finish on the encoders.
                    dist = LOST_CREEP if LOST_CREEP is not None else max(0.0, last_r - STOP_RANGE)
                    self.creep(dist)
                    print(f"[FOUND VICTIM] lost from view at {last_r * 100:.0f} cm - "
                          f"drove the last {dist * 100:.0f} cm")
                    return True
                lost_since = lost_since or tick
                if tick - lost_since > LOST_TIME:
                    return False
                time.sleep(period)
                continue
            lost_since = None
            r, b = victim                                   # b > 0: to the right
            last_r = r if abs(b) < math.radians(15) else None
            if r <= STOP_RANGE:
                self.hw.stop()
                print(f"[FOUND VICTIM] stopped {r * 100:.1f} cm away")
                return True
            if abs(b) > AIM_TOLERANCE:
                turn, forward = -clamp(TURN_GAIN * b, TURN_SPEED), APPROACH_SPEED * 0.4
            else:
                turn, forward = 0.0, APPROACH_SPEED
            self.hw.drive(forward, W_LEFT * turn)
            time.sleep(max(0.0, period - (time.monotonic() - tick)))


# ===========================================================================
# Mission: map, explore, rescue, return
# ===========================================================================

class Mission:
    def __init__(self, hw):
        self.hw = hw
        self.drv = Driver(hw)
        self.pos = BASE
        self.known = {}             # cell -> set of open directions
        self.dead_ends = set()
        self.blocked = set()        # (cell, direction) found blocked while moving
        self.victim_cell = None
        self.path_home = []
        self.traversed = set()      # passages already driven - known open, never marked blocked
        self.collected = False      # Rescue Collection confirmed the pickup
        self.cell_time = CELL_TIME  # measured average seconds per cell
        self.start = time.monotonic()

    def elapsed(self):
        return time.monotonic() - self.start

    # --- map -------------------------------------------------------------

    def sense(self, came_from=None):
        """Record which sides of this cell are open."""
        if self.pos in self.known:
            return
        f = self.drv.facing
        left, right = self.hw.left_m(), self.hw.right_m()
        front = self.drv.front_clearance()
        opened = set()
        if front > FRONT_WALL:
            opened.add(f)
        if left is None or left > SIDE_WALL:
            opened.add(left_of(f))
        if right is None or right > SIDE_WALL:
            opened.add(right_of(f))
        if came_from:
            opened.add(came_from)
        self.known[self.pos] = opened
        if len(opened) == 1 and self.pos != BASE:
            self.dead_ends.add(self.pos)
        print(f"[MAP] {self.pos} facing {f}: L {fmt(left)} F {fmt(None if front > 50 else front)} R {fmt(right)}"
              f" -> open {''.join(d for d in ORDER if d in opened)}"
              f"{'  (dead end)' if self.pos in self.dead_ends else ''}")
        self.print_map()

    def mark_blocked(self, d):
        self.blocked.add((self.pos, d))
        self.known.get(self.pos, set()).discard(d)
        n = step(self.pos, d)
        if n in self.known:
            self.known[n].discard(OPPOSITE[d])

    def bfs(self, goal=None):
        """Shortest path through the map to `goal`, or to the nearest
        unexplored opening if goal is None. Prefers carrying on straight."""
        f = self.drv.facing
        pref = [f, left_of(f), right_of(f), OPPOSITE[f]]
        prev = {self.pos: None}
        q = deque([self.pos])
        found = None
        while q:
            c = q.popleft()
            if goal is not None and c == goal:
                found = c
                break
            for d in pref:
                if d not in self.known.get(c, ()):
                    continue
                n = step(c, d)
                if n in prev:
                    continue
                prev[n] = c
                if goal is None and n not in self.known:
                    found = n
                    q.clear()
                    break
                q.append(n)
        if found is None:
            return None
        path = [found]
        while path[-1] != self.pos:
            path.append(prev[path[-1]])
        return path[::-1]

    def step_to(self, nxt, look=False):
        """Move one cell. With look=True, after turning to face the new cell
        it checks for a victim first - a victim in a side dead end only
        comes into view once the robot faces it, and driving in would put
        the robot on top of it, out of the camera's view. Returns
        "rescued", True (moved) or False (blocked)."""
        d = next(k for k, v in DIRS.items() if v == (nxt[0] - self.pos[0], nxt[1] - self.pos[1]))
        if look and d != self.drv.facing:
            self.drv.face(d)
            if self.try_rescue():
                return "rescued"
        edge = frozenset((self.pos, nxt))
        t0 = time.monotonic()
        for attempt in range(3 if edge in self.traversed else 2):
            self.drv.face(d)
            if self.drv.forward_cell():
                if attempt == 0:            # learn how long a cell really takes
                    self.cell_time = 0.7 * self.cell_time + 0.3 * (time.monotonic() - t0)
                self.traversed.add(edge)
                self.pos = nxt
                self.drv.face(d)            # square up: wall steering can leave it angled
                new_cell = nxt not in self.known
                self.sense(came_from=OPPOSITE[d])
                if look and new_cell and PEEK_SIDES and self.peek_sides():
                    return "rescued"
                return True
            print(f"[MOVE] {self.pos} -> {nxt} failed, retrying")
            # re-square and re-centre from this cell's walls before trying again
            self.drv.square_up_on_walls()
            self.drv.face(d)
            self.drv.recentre()
        if edge in self.traversed:
            # Driven before, so it IS open - never wipe the way home off the map
            print(f"[MOVE] {self.pos} -> {nxt} keeps failing on a path already driven - will retry")
            return False
        print(f"[MOVE] {self.pos} -> {nxt} blocked - marking it on the map")
        self.mark_blocked(d)
        return False

    def peek_sides(self):
        """Turn to look down each unexplored side opening of this cell for a
        victim, then face the way it was going. True if one was rescued."""
        f = self.drv.facing
        for d in (left_of(f), right_of(f)):
            if d in self.known.get(self.pos, ()) and step(self.pos, d) not in self.known:
                print(f"[PEEK] looking {d} down the side opening")
                self.drv.face(d)
                if self.try_rescue():
                    return True
        if self.drv.facing != f:
            self.drv.face(f)
        return False

    # --- victim -----------------------------------------------------------

    def try_rescue(self):
        """If a victim is in view, close on it, collect it and back out to
        this cell. True once collected."""
        victim = self.hw.look()[0] or self.hw.look()[0]     # second frame in case one flickers
        if victim is None:
            return False
        r, b = victim
        if r > SEEK_RANGE or abs(b) > SEEK_BEARING:
            return False
        print(f"[DETECT] victim {r:.2f} m, {math.degrees(b):+.0f} deg - approaching")
        self.hw.lower_clamp()           # clamp down while closing on it
        x0, y0, _ = self.drv.pose()
        if not self.drv.approach():
            print("[LOST] victim out of view - back to exploring")
            self.hw.raise_clamp()       # false alarm: clamp back up
            self.drv.back_to(x0, y0)
            self.drv.face(self.drv.facing)
            return False
        self.hw.led("green", True)
        x1, y1, _ = self.drv.pose()
        cells = max(1, round(math.hypot(x1 - x0, y1 - y0) / CELL + 0.3))
        self.victim_cell = self.pos
        for _ in range(cells):
            self.victim_cell = step(self.victim_cell, self.drv.facing)
        for attempt in range(1, (MAX_COLLECT_TRIES if USE_RESCUE else 0) + 1):
            print("[COLLECT] commanding Rescue Collection")
            ok, label = self.hw.collect()
            if ok:
                print(f"[COLLECTED] {label}")
                self.collected = True
                break
            print(f"[COLLECT] attempt {attempt} failed")
        else:
            print("[COLLECT] no rescue hardware fitted - skipping collection, returning to base"
                  if not USE_RESCUE else "[COLLECT] no confirmation - returning to base anyway")
        # back out to the cell centre the approach started from
        self.drv.back_to(x0, y0)
        self.drv.face(self.drv.facing)
        return True

    # --- phases -------------------------------------------------------------

    def explore(self):
        print("[EXPLORE] leaving base zone")
        self.sense()
        for _ in range(200):
            if self.try_rescue():
                return True
            home = self.bfs(goal=BASE)
            needed = (len(home) - 1) * self.cell_time * 1.3 + HOME_MARGIN if home else HOME_MARGIN
            if self.elapsed() + needed > TIME_LIMIT:
                print(f"[TIME] {TIME_LIMIT - self.elapsed():.0f} s left, ~{needed:.0f} s needed "
                      "to get home - returning to base")
                return False
            path = self.bfs()
            if path is None:
                print("[EXPLORE] whole reachable maze explored - no victim found")
                return False
            for nxt in path[1:]:
                moved = self.step_to(nxt, look=True)
                if moved == "rescued":
                    return True
                if not moved:
                    break
                if self.try_rescue():
                    return True
        return False

    def go_home(self):
        for _ in range(50):
            if self.pos == BASE:
                return True
            if self.elapsed() > TIME_LIMIT * 1.5:
                print("[RETURN] well over time - stopping")
                return False
            path = self.bfs(goal=BASE)
            if path is None:
                print("[RETURN] no known route to base")
                return False
            self.path_home = path
            print(f"[RETURN] route home: {' -> '.join(map(str, path))}")
            self.print_map()
            for nxt in path[1:]:
                if not self.step_to(nxt):
                    break
        return self.pos == BASE

    # --- evidence: the map as a drawing --------------------------------------

    def print_map(self):
        """Explored maze as walls and passages. B base, R robot, V victim,
        D dead end, * route home, ? not explored yet."""
        cells = set(self.known) | {self.pos}
        xs = [c[0] for c in cells]
        ys = [c[1] for c in cells]
        route = set(self.path_home)
        lines = []

        def is_open(c, d):
            n = step(c, d)
            return d in self.known.get(c, ()) or OPPOSITE[d] in self.known.get(n, ())

        for y in range(max(ys), min(ys) - 1, -1):
            top, mid = "", ""
            for x in range(min(xs), max(xs) + 1):
                c = (x, y)
                open_ = self.known.get(c)
                top += "+" + ("   " if is_open(c, "N") else "---")
                wall = " " if is_open(c, "W") else "|"
                if c == self.pos:
                    mark = "R"
                elif c == BASE:
                    mark = "B"
                elif c == self.victim_cell:
                    mark = "V"
                elif c in route:
                    mark = "*"
                elif c in self.dead_ends:
                    mark = "D"
                elif open_ is None:
                    mark = "?"
                else:
                    mark = " "
                mid += wall + f" {mark} "
            mid += " " if is_open((max(xs), y), "E") else "|"
            lines += [top + "+", mid]
        lines.append("+---" * (max(xs) - min(xs) + 1) + "+")
        print("\n".join(lines))


def run():
    hw = Hardware()
    mission = Mission(hw)
    try:
        if USE_RESCUE:
            if not hw.rescue.ping():
                print("[RESCUE] WARNING: ESP32 not answering PING - collection will fail")
            hw.raise_clamp()            # idle while exploring
        found = mission.explore()
        home = mission.go_home()
        if found and home and USE_RESCUE:
            print("[BASE] in the base zone - releasing the victim")
            hw.release()
        status = ("victim collected" if mission.collected else
                  "victim reached (no rescue hardware fitted)" if found and not USE_RESCUE else
                  "victim reached, collection NOT confirmed" if found else "victim not found")
        print(f"[DONE] {status}, {'back at base' if home else 'NOT back at base'}, "
              f"{mission.elapsed():.0f} s")
        mission.print_map()
    finally:
        hw.shutdown()


# ===========================================================================
# Checks and calibration
# ===========================================================================

def check():
    """Hardware check before every run. Prints PASS / FAIL for each part."""
    hw = Hardware()
    drv = Driver(hw)
    try:
        print("LEDs on for 1 s - watch them:")
        for name in hw.leds:
            hw.led(name, True)
        time.sleep(1.0)
        for name in hw.leds:
            hw.led(name, False)

        front = lambda: (lambda d: None if d > 50 else d)(hw.front_m())
        for side, read in (("LEFT", hw.left_m), ("RIGHT", hw.right_m), ("FRONT", front)):
            input(f"Hold your hand ~10 cm from the {side} ultrasonic, then press Enter...")
            vals = []
            for _ in range(10):
                vals.append(read())
                time.sleep(0.2)
            seen = [v for v in vals if v is not None]
            good = any(v < 0.3 for v in seen)
            print(f"  {side}: {' '.join(fmt(v) for v in vals)}")
            print(f"  {'PASS' if good else 'FAIL - no echo: check TRIG/ECHO pins, 5V/GND and the echo divider'}")

        print("Forward 1 s (battery must be connected):")
        hw.encoders()
        hw.drive(DRIVE_SPEED, 0.0)
        time.sleep(1.0)
        hw.stop()
        time.sleep(0.3)
        dl, dr = hw.encoders()
        ml, mr = dl / TICKS_PER_M * 1000, dr / TICKS_PER_M * 1000
        if abs(ml) < 2 and abs(mr) < 2:
            print("  wheels didn't move - is the battery connected? Skipping the turn test.")
        else:
            good = ml > 10 and mr > 10 and abs(ml - mr) < 0.3 * max(ml, mr)
            print(f"  left {ml:.0f} mm  right {mr:.0f} mm  (expect ~{DRIVE_SPEED * 1000:.0f} each)  "
                  f"{'PASS' if good else 'FAIL - wheels/encoders: check mobility.py with Mobility'}")

            print("Turning LEFT 90 deg on the encoders:")
            _, _, h0 = drv.pose()
            ok = drv.turn_to(wrap(h0 + math.pi / 2), timeout=8.0)
            _, _, h1 = drv.pose()
            print(f"  encoders say {math.degrees(wrap(h1 - h0)):+.0f} deg.  "
                  "Did it physically turn LEFT about 90 deg?")
            print("  " + ("PASS if yes. If it turned RIGHT, the left/right motors are swapped in "
                          "mobility.py." if ok else
                          "FAIL - the turn never finished: motors and encoders disagree. "
                          "Fix in mobility.py (motor or encoder direction)."))
            drv.turn_to(h0, timeout=8.0)

        victim, _ = hw.look()
        if victim:
            print(f"Vision: victim {victim[0] * 100:.0f} cm at {math.degrees(victim[1]):+.0f} deg")
        else:
            print("Vision: no victim in view")
        if USE_RESCUE:
            if hw.rescue.ping():
                print("Rescue link: PASS (PONG from the ESP32)")
                print("  clamp: LOWER (95 deg), then RAISE (idle) - watch it move")
                hw.lower_clamp()
                time.sleep(1.5)
                hw.raise_clamp()
                time.sleep(1.5)
            else:
                print("Rescue link: FAIL - no PONG. Check Pi TX -> ESP32 GPIO3, Pi RX <- "
                      "ESP32 GPIO1, common GND, and that the sketch is uploaded")
        else:
            print("Rescue link: not fitted (USE_RESCUE = False)")
    finally:
        hw.shutdown()


def look():
    """Live view of what navigation sees, no driving. Ctrl+C (or q in the
    window) to stop.

    Shows Vision's camera picture with its boxes, and navigation's
    readings (victim from the camera, walls from the three ultrasonics). With no screen (plain
    SSH) it saves the latest picture to nav_look.jpg instead.

    For LEFT/RIGHT_TARGET_CM put the robot centred in a corridor; for
    FRONT_CENTRED_CM centred in a cell facing a wall.

    Also use it to find where the camera LOSES the victim: slide a token
    towards the robot and note the range just before 'no victim' appears.
    LOST_CLOSE should sit a little above that."""
    import cv2
    hw = Hardware()
    odo = Odometry(hw)
    screen = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    snapshot = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_look.jpg")
    if not screen:
        print(f"No screen found - saving the camera view to {snapshot} (open it over VNC "
              "or copy it off with scp). Readings print below.")
    try:
        while True:
            odo.update()
            victim, _ = hw.look(draw=True)
            left, right, clearance = hw.left_m(), hw.right_m(), hw.front_m()
            v = (f"victim {victim[0] * 100:5.1f}cm {math.degrees(victim[1]):+5.1f}deg"
                 if victim else "no victim")
            wall = "nothing" if clearance > 50 else f"{clearance * 100:5.1f}cm"
            line1 = f"{v}"
            line2 = f"left {fmt(left)}   front {wall}   right {fmt(right)}"
            print(f"{line1}   {line2}   odo x {odo.x * 100:+6.1f}cm y {odo.y * 100:+6.1f}cm "
                  f"heading {math.degrees(odo.th):+6.1f}deg")

            frame = getattr(hw.vision, "last_frame", None)
            if frame is not None:
                h, w = frame.shape[:2]
                for i, text in enumerate((line1, line2)):
                    cv2.putText(frame, text, (30, h - 110 + 60 * i), cv2.FONT_HERSHEY_SIMPLEX,
                                1.6, (0, 0, 0), 8)
                    cv2.putText(frame, text, (30, h - 110 + 60 * i), cv2.FONT_HERSHEY_SIMPLEX,
                                1.6, (255, 255, 255), 3)
                small = cv2.resize(frame, (w // 2, h // 2))
                if screen:
                    cv2.imshow("Navigation view (q to quit)", small)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    cv2.imwrite(snapshot, small)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if screen:
            cv2.destroyAllWindows()
        hw.shutdown()


def ticks():
    """Drive forward 30 cm (6 s) and print how far the encoders say it went.
    Measure the real distance: if they differ, Mobility's TICKS_PER_MM
    needs scaling by (encoder distance / real distance)."""
    hw = Hardware()
    try:
        input("Clear 40 cm ahead, mark where the front of the robot is, press Enter...")
        hw.encoders()
        tl = tr = 0
        hw.drive(DRIVE_SPEED, 0.0)
        end = time.monotonic() + 6.0
        while time.monotonic() < end:
            time.sleep(0.05)
            dl, dr = hw.encoders()
            tl += LEFT_TICK_SIGN * dl
            tr += RIGHT_TICK_SIGN * dr
        hw.stop()
        time.sleep(0.3)
        dl, dr = hw.encoders()
        tl += LEFT_TICK_SIGN * dl
        tr += RIGHT_TICK_SIGN * dr
        print(f"commanded: {DRIVE_SPEED * 6 * 1000:.0f} mm")
        print(f"encoders:  left {tl / TICKS_PER_M * 1000:.0f} mm  right {tr / TICKS_PER_M * 1000:.0f} mm")
        print("Measure the real distance. If it differs, multiply TICKS_PER_MM in "
              "mobility.py by (encoder mm / real mm).")
    finally:
        hw.shutdown()


def spin():
    """Turn left until the encoders say 360 deg. If it actually turned A deg,
    set TRACK = TRACK * 360 / A."""
    hw = Hardware()
    drv = Driver(hw)
    try:
        input("Mark which way the robot faces, press Enter...")
        total, last = 0.0, drv.pose()[2]
        hw.drive(0.0, W_LEFT * TURN_SPEED)
        while total < 2 * math.pi:
            time.sleep(0.05)
            th = drv.pose()[2]
            total += wrap(th - last)
            last = th
        hw.stop()
        print("Encoders say 360 deg. How far did it really turn? If not 360, set "
              "TRACK = mobility.WHEELBASE * 360 / (degrees turned)")
    finally:
        hw.shutdown()


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--look" in sys.argv:
        look()
    elif "--ticks" in sys.argv:
        ticks()
    elif "--spin" in sys.argv:
        spin()
    else:
        run()
