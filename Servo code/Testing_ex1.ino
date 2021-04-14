/*
  Servo Code testing
*/
#include <Wire.h>
#include <Servo.h>

Servo servo_x;

int angle = 0;

void setup()
{
  servo_x.attach(9);
  servo_x.writeMicroseconds(1500);  // set servo to mid-point
}

void loop()
{
  for (angle = 0; angle < 60; angle += 1) // command to move from -60 degrees to 60 degrees
  {
    servo_x.write(angle);
    delay(100);
  }
  for (angle = 60; angle >= -60; angle -= 1) // command to move from 60 degrees to -60 degrees
  {
    servo_x.write(angle);
    delay(100);
  }
}
