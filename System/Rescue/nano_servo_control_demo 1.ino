/*
  Nano servo controller — receives commands over USB serial from a Raspberry Pi.

  Protocol (newline-terminated ASCII):
    S<angle>\n   -> move servo to <angle> degrees (0-180), e.g. "S97\n"

  Responses:
    READY        -> sent once at boot, once Serial + Servo are initialised
    OK           -> command accepted and the settle delay has elapsed
                    (i.e. the servo should physically be at the new angle)
    ERR <reason> -> bad command (unparsable, out of range, etc.)

  Sequencing: the Nano is single-threaded and blocking by design here — it
  won't read the next line off Serial until it has finished handling (and
  responded to) the current one. That means "wait for OK before sending the
  next command" on the Pi side is what actually keeps servo-move and
  motor-move steps from overlapping. Don't send commands without waiting for
  the reply, or you'll queue moves up in the serial buffer and lose the
  synchronisation this whole thing depends on.
*/

#include <Servo.h>

Servo myServo;
const int SERVO_PIN = 9;

// Update this to match wherever your servo actually sits at power-on /
// attach() — if it doesn't match reality, your very first settle-time
// calculation will be wrong (though only by one move, since currentAngle
// gets corrected after that).
int currentAngle = 90;

// Rough settle-time estimate. Typical 9g/MG90S-class hobby servos do
// roughly 0.1s per 60 degrees unloaded — this uses ~4ms/degree plus a
// fixed overhead, which is deliberately generous. If your servo is
// slower/loaded (e.g. it's lifting or holding something), bump the
// multiplier up until moves look complete before OK is sent.
unsigned long settleTimeMs(int fromAngle, int toAngle) {
  int delta = abs(toAngle - fromAngle);
  return (unsigned long)(delta * 4) + 50;
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);
  Serial.println("READY");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line);
    }
  }
}

void handleCommand(const String &line) {
  if (line.length() < 2 || (line[0] != 'S' && line[0] != 's')) {
    Serial.println("ERR unknown command");
    return;
  }

  String numPart = line.substring(1);
  numPart.trim();

  bool isNumeric = numPart.length() > 0;
  for (unsigned int i = 0; i < numPart.length(); i++) {
    if (!isDigit(numPart[i])) {
      isNumeric = false;
      break;
    }
  }

  if (!isNumeric) {
    Serial.println("ERR angle not numeric");
    return;
  }

  int angle = numPart.toInt();
  if (angle < 0 || angle > 180) {
    Serial.println("ERR angle out of range");
    return;
  }

  unsigned long wait = settleTimeMs(currentAngle, angle);
  myServo.write(angle);
  currentAngle = angle;
  delay(wait); // blocking is fine — this Nano has exactly one job

  Serial.println("OK");
}
