"""
Interface check: Navigation -> Rescue Collection
================================================
Verifies the serial link to the Nano and that it speaks the agreed
protocol.

    python3 check_rescue.py           run the checks
    python3 check_rescue.py --listen  dump raw bytes, for debugging

THE PROTOCOL - this is the contract between the two subsystems
    Pi  -> Nano   "PING\\n"          ->   "PONG\\n"
    Pi  -> Nano   "COLLECT\\n"       ->   "OK <label>\\n"   collected
                                          "FAIL <why>\\n"   attempted, failed

Nav treats anything else, or silence, as a failed collection.

IF THE NANO HAS NO SKETCH YET, this will report no response - that is
the expected result, not a fault in the wiring. Hand the protocol above
to whoever owns Rescue Collection.
"""

import sys
import time

try:
    import serial
except ImportError:
    print("pyserial missing - pip install pyserial")
    sys.exit(1)

PORTS = ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/serial0"]
BAUD = 115200


def open_link():
    """Try each candidate port. USB Nanos appear as ttyUSB0 or ttyACM0
    depending on whether the chip is a clone; serial0 is the GPIO UART."""
    for port in PORTS:
        try:
            ser = serial.Serial(port, BAUD, timeout=0.5)
        except Exception:
            continue
        print(f"port opened: {port}")
        print("  (this only means the PI's port exists - it says nothing")
        print("   about whether a Nano is actually on the other end)")
        time.sleep(2.0)     # Nano resets when the port opens
        ser.reset_input_buffer()
        return ser, port
    print(f"FAIL: none of {PORTS} could be opened")
    print("  USB: is the cable plugged in? check with  ls /dev/tty*")
    print("  GPIO UART: enabled?  sudo raspi-config > Interface > Serial")
    return None, None


def ask(ser, cmd, timeout):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            return line
    return None


def run_checks():
    ser, port = open_link()
    if ser is None:
        return

    try:
        print("\n1. PING")
        reply = ask(ser, "PING", 3.0)
        if reply is None:
            print("   no response")
            print("   - has the Nano been flashed with a sketch that reads")
            print("     serial and replies? that is the most likely cause")
            print("   - GPIO UART: Nano TX is 5V and the Pi is 3.3V only,")
            print("     so that direction needs a divider or level shifter")
            print("   - does the sketch use Serial.begin(115200)?")
        elif reply == "PONG":
            print("   PONG - link good, protocol understood")
        else:
            print(f"   replied {reply!r}, expected 'PONG'")
            print("   something is listening but not speaking the protocol")

        print("\n2. COLLECT")
        print("   the mechanism should physically actuate now")
        reply = ask(ser, "COLLECT", 10.0)
        if reply is None:
            print("   no response within 10 s")
            print("   nav will treat this as a failed collection")
        elif reply.startswith("OK"):
            label = reply[2:].strip() or "(no label)"
            print(f"   OK - reported collected: {label}")
            print("   NOTE: does the Nano actually VERIFY a victim is held,")
            print("   or does it send OK whenever the servo runs? If there")
            print("   is no switch or sensor, OK only means 'servo moved'.")
        elif reply.startswith("FAIL"):
            print(f"   {reply} - reported failure, handled correctly by nav")
        else:
            print(f"   replied {reply!r}, expected 'OK ...' or 'FAIL ...'")

        print("\n3. Unknown command")
        reply = ask(ser, "XYZZY", 2.0)
        print(f"   replied {reply!r}")
        print("   ideally the Nano ignores anything it does not recognise,")
        print("   so line noise cannot trigger the gripper")

    finally:
        ser.close()


def listen():
    """Raw dump. Useful when the Nano replies but nav does not like it."""
    ser, port = open_link()
    if ser is None:
        return
    print("\nRaw bytes for 20 s. Ctrl+C to stop early.\n")
    try:
        end = time.monotonic() + 20
        while time.monotonic() < end:
            data = ser.read(200)
            if data:
                print(repr(data))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    if "--listen" in sys.argv:
        listen()
    else:
        run_checks()
