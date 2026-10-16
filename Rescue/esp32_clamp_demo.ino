/*
  EGB320 Rescue Collection - ESP32 clamp controller, GPIO signal link
  Board: ESP32-S 30-pin dev board  ("ESP32 Dev Module" in Arduino IDE)

  No serial link to the Pi. Three plain signal wires + ground:
    Pi pin 36 (GPIO16)  -> ESP32 GPIO16   command bit 0
    Pi pin 38 (GPIO20)  -> ESP32 GPIO17   command bit 1
    Pi pin 40 (GPIO21)  <- ESP32 GPIO26   DONE
    Pi GND (pin 34/39)  -- ESP32 GND
    Servo signal -> ESP32 GPIO18; servo power from its own 5-6 V supply,
    GND shared with the ESP32. Both boards are 3.3 V logic.

  Command code = bit1 bit0:
    00  IDLE     clamp to idle   (RAISE)
    01  OPEN     clamp open      (LOWER)
    10  CLAMP    open, then clamp on the victim  (COLLECT)
    11  RELEASE  open in the base zone           (RELEASE)

  DONE: LOW while the servo is moving, HIGH when finished / idle.
        Held LOW for at least 100 ms per command so the Pi always sees it.
  The ESP32 only LISTENS: it does nothing on its own. It acts on a code
  only when the Pi changes it and it has been steady for 30 ms (the Pi
  sets the two bits a moment apart). The servo isn't even switched on
  until the first command, so at power-up the clamp doesn't move.

  USB Serial is free for debug prints - they don't go to the Pi.
  Library: "ESP32Servo" (Arduino Library Manager).
*/

#include <ESP32Servo.h>

const int SERVO_PIN = 18;
const int CMD0_PIN  = 16;   // input, bit 0 (free on WROOM boards like the ESP32-S)
const int CMD1_PIN  = 17;   // input, bit 1
const int DONE_PIN  = 26;   // output to the Pi

// ---- servo angles, degrees ----
const int ANGLE_OPEN  = 146;  // lowered / open, ready to take the victim
const int ANGLE_CLAMP = 95;   // clamped on the victim
const int ANGLE_IDLE  = 95;   // travelling with no victim in sight

const int MOVE_STEP_MS = 12;  // ms per degree - slower = gentler on the token
const int SETTLE_MS    = 300; // pause after each move
const int STEADY_MS    = 30;  // code must be steady this long before acting
const int MIN_BUSY_MS  = 100; // DONE stays LOW at least this long

Servo clamp;
bool servoOn = false;      // servo only switched on by the first command
int currentAngle = -1;     // unknown until the first command
int lastCode = 0;

int readCode() {
  return (digitalRead(CMD1_PIN) << 1) | digitalRead(CMD0_PIN);
}

void moveTo(int target) {
  if (!servoOn) {                       // first command: switch the servo on
    clamp.setPeriodHertz(50);
    clamp.attach(SERVO_PIN, 500, 2400);
    servoOn = true;
  }
  if (currentAngle < 0) {               // position unknown - go straight there
    currentAngle = target;
    clamp.write(target);
    delay(SETTLE_MS);
    return;
  }
  int step = (target > currentAngle) ? 1 : -1;
  while (currentAngle != target) {
    currentAngle += step;
    clamp.write(currentAngle);
    delay(MOVE_STEP_MS);
  }
  delay(SETTLE_MS);
}

void act(int code) {
  const char *names[] = {"IDLE", "OPEN", "CLAMP", "RELEASE"};
  Serial.print("[cmd] ");
  Serial.println(names[code]);
  switch (code) {
    case 0: moveTo(ANGLE_IDLE); break;
    case 1: moveTo(ANGLE_OPEN); break;
    case 2: moveTo(ANGLE_OPEN); moveTo(ANGLE_CLAMP); break;
    case 3: moveTo(ANGLE_OPEN); break;
  }
}

void setup() {
  Serial.begin(115200);                     // debug only
  pinMode(CMD0_PIN, INPUT_PULLDOWN);        // Pi unplugged reads as 00 = idle
  pinMode(CMD1_PIN, INPUT_PULLDOWN);
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW);              // busy while starting up
  delay(200);
  lastCode = readCode();                    // whatever the Pi holds now is not a new command
  digitalWrite(DONE_PIN, HIGH);             // ready - now just listening
  Serial.println("clamp controller listening - GPIO link (cmd 16/17, done 26)");
}

void loop() {
  int code = readCode();
  if (code != lastCode) {
    delay(STEADY_MS);
    if (readCode() == code) {               // steady: act on it
      digitalWrite(DONE_PIN, LOW);
      unsigned long started = millis();
      act(code);
      while (millis() - started < (unsigned long)MIN_BUSY_MS) {}
      lastCode = code;
      digitalWrite(DONE_PIN, HIGH);
    }
  }
  delay(5);
}
