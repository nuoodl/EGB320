#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 18;

void setup()
{
    Serial.begin(115200);

    myServo.attach(SERVO_PIN);

    myServo.write(120);

    Serial.println("Servo ready!");
    Serial.println("Enter angle from 0 to 180:");
}

void loop()
{
    if (Serial.available() > 0)
    {
        int angle = Serial.parseInt();

        if (angle >= 0 && angle <= 180)
        {
            myServo.write(angle);

            Serial.print("Servo set to: ");
            Serial.print(angle);
            Serial.println(" degrees");
        }
        else
        {
            Serial.println("Invalid angle. Enter 0-180.");
        }
    }
}
