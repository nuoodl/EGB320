/*
  EGB320 Rescue Collection - ESP32 clamp controller
  Board: ESP32-S 30-pin dev board  ("ESP32 Dev Module" in Arduino IDE)

  The Pi link uses the ESP32's DEFAULT serial port (UART0):
    Pi TX  (GPIO14, pin 8)   -> ESP32 RX  = GPIO3
    Pi RX  (GPIO15, pin 10)  <- ESP32 TX  = GPIO1
    Pi GND                   -- ESP32 GND
    Servo signal             -> ESP32 GPIO18
    Servo power              -> its own 5-6 V supply, GND shared with ESP32.
  Both sides are 3.3 V logic - no level shifter.

  !! GPIO1/GPIO3 are ALSO the USB programming port. So:
     - UNPLUG the Pi TX/RX wires from the ESP32 while uploading, or the
       upload fails (and the Pi's TX pin fights the USB chip).
     - Don't run the USB cable to a laptop while the Pi is connected.
     - Nothing else may print to Serial - any debug text would go to the Pi.
     - On reset the ESP32 prints boot text at 115200. The Pi code clears
       its input buffer before every command, so it's harmless.

  Protocol (matches RescueLink in nav_hd_demo.py), 115200 baud, newline ended:
    PING      -> PONG
    LOWER     clamp to 95 deg (open). SILENT.
    RAISE     clamp to idle. SILENT.
    COLLECT   open (95) then clamp (40)  -> OK victim
    RELEASE   open (95) in the base zone -> OK released
    other     -> FAIL unknown

  Library: "ESP32Servo" (Arduino Library Manager).
*/

#include <ESP32Servo.h>

const int SERVO_PIN = 18;

// ---- servo angles, degrees ----
const int ANGLE_OPEN  = 95;   // lowered / open, ready to take the victim
const int ANGLE_CLAMP = 40;   // clamped on the victim
const int ANGLE_IDLE  = 40;   // travelling with no victim in sight.
                              // CHANGE THIS if idle should be a different angle.

const int MOVE_STEP_MS = 12;  // ms per degree - slower = gentler on the token
const int SETTLE_MS    = 300; // pause after each move

Servo clamp;
int currentAngle = ANGLE_IDLE;
String line;

void moveTo(int target) {
  int step = (target > currentAngle) ? 1 : -1;
  while (currentAngle != target) {
    currentAngle += step;
    clamp.write(currentAngle);
    delay(MOVE_STEP_MS);
  }
  delay(SETTLE_MS);
}

void handle(const String &cmd) {
  if (cmd == "PING") {
    Serial.println("PONG");
  } else if (cmd == "LOWER") {
    moveTo(ANGLE_OPEN);                 // silent on purpose
  } else if (cmd == "RAISE") {
    moveTo(ANGLE_IDLE);                 // silent on purpose
  } else if (cmd == "COLLECT") {
    moveTo(ANGLE_OPEN);                 // no-op if LOWER already did it
    moveTo(ANGLE_CLAMP);
    Serial.println("OK victim");
  } else if (cmd == "RELEASE") {
    moveTo(ANGLE_OPEN);
    Serial.println("OK released");
  } else {
    Serial.println("FAIL unknown");
  }
}

void setup() {
  Serial.begin(115200);                  // the Pi link (GPIO1 TX / GPIO3 RX)
  clamp.setPeriodHertz(50);
  clamp.attach(SERVO_PIN, 500, 2400);
  clamp.write(ANGLE_IDLE);               // start in idle
  delay(500);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length()) handle(line);
      line = "";
    } else if (c != '\r') {
      line += c;
      if (line.length() > 32) line = "";   // junk guard
    }
  }
}
