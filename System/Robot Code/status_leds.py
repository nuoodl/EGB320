"""
EGB320 status LEDs - driven directly from Raspberry Pi 5 GPIO
=============================================================
No microcontroller in the middle. Three LEDs on three GPIO pins.

WHY gpiozero AND NOT RPi.GPIO
    RPi.GPIO does not work on the Raspberry Pi 5. The Pi 5 moved its GPIO
    behind the new RP1 southbridge and RPi.GPIO was never updated for it,
    so it errors out or silently does nothing. gpiozero is the supported
    library and ships with Raspberry Pi OS Bookworm. If it is missing:
        sudo apt install python3-gpiozero

PINS (BCM numbering, not physical pin numbers)
    GPIO17  yellow   physical pin 11
    GPIO4   green    physical pin 7
    GPIO22  red      physical pin 15
    Any GND, e.g. physical pin 9, 14, 20, 25

    These three are chosen because nothing else wants them. Do NOT move
    the LEDs onto GPIO2/3 (I2C - your motor controller board uses that)
    or GPIO14/15 (UART), or you will break those buses.

WIRING per LED
    GPIO pin -> resistor -> LED anode (long leg)
    LED cathode (short leg) -> GND
    220 ohm is a safe starting point. If the LEDs look too dim to satisfy
    "clearly visible from any viewing angle", drop to 150 ohm rather than
    removing the resistor. Do not go below ~120 ohm: a Pi GPIO pin should
    not be asked for much more than 16 mA, and all pins together should
    stay well under ~50 mA.

    If they still are not bright enough, that is a sign you need a
    transistor driving each LED from the 5V rail rather than powering it
    from the GPIO pin itself. Worth raising with whoever owns the
    electrical side.

USE
    from status_leds import StatusLEDs

    leds = StatusLEDs()
    leds.searching()    # yellow - exploring the maze
    leds.found()        # green  - victim found, collecting
    leds.returning()    # red    - carrying a victim back
    leds.off()

Run this file directly to test the wiring.
"""

import time

try:
    from gpiozero import LED
except ImportError:
    LED = None


# BCM numbering. Change here if you wire different pins.
LED_PINS = {
    "yellow": 17,
    "green": 4,
    "red": 22,
}


class StatusLEDs:
    """Exactly one LED lit at a time - the assessment grades whether the
    LED state matches what the robot is actually doing, so overlapping
    colours would be ambiguous.

    Never raises once constructed. If an LED call fails mid-demo you want
    the robot to carry on, not die on a status indicator, so failures are
    swallowed and reported once."""

    def __init__(self):
        if LED is None:
            raise RuntimeError(
                "gpiozero not installed - sudo apt install python3-gpiozero"
            )
        self.leds = {name: LED(pin) for name, pin in LED_PINS.items()}
        self._warned = False

    def _show(self, colour):
        try:
            for name, led in self.leds.items():
                if name == colour:
                    led.on()
                else:
                    led.off()
        except Exception as exc:
            if not self._warned:
                print(f"[LED] failed ({exc}) - continuing without LEDs")
                self._warned = True

    def searching(self):
        self._show("yellow")

    def found(self):
        self._show("green")

    def returning(self):
        self._show("red")

    def off(self):
        self._show(None)

    def test(self):
        """Each LED in turn, then all three. Confirms wiring."""
        for name in ("yellow", "green", "red"):
            print(f"  {name}")
            self._show(name)
            time.sleep(0.8)
        try:
            for led in self.leds.values():
                led.on()
            time.sleep(0.8)
        except Exception:
            pass
        self.off()

    def close(self):
        self.off()
        try:
            for led in self.leds.values():
                led.close()
        except Exception:
            pass


if __name__ == "__main__":
    print("Testing status LEDs...")
    leds = StatusLEDs()
    try:
        leds.test()
        print("Done - if each LED lit in turn, the wiring is good.")
    finally:
        leds.close()
