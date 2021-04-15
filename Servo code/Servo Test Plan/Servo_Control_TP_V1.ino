/*
 * 90 degrees = Center position
 * 30 degrees = Max angle to the right
 * 150 degrees = Max angle to the left
 * Servo has a total movement of 120 degrees
*/

#ifndef _SERVOCONTROL_H
#define _SERVOCONTROL_H

#include <Wire.h>
#include <Servo.h>

Servo servo_x;

int angle = 0; // Setting a angle to 0 doesn't mean it's at 0deg it means initial position
int angle_30deg = 30;
int center_position = 90;

void Servo_setup()
{
  servo_x.attach(9);
  servo_x.write(angle_30deg);
}

void Servo_Movement()
{
  for (angle = 0; angle <= 150; angle += 1) // command to move from 90 degrees to 150 degrees
  {
    servo_x.write(angle);
    delay(15);
  }
  delay(1000);

  for (angle = 150; angle >= 30; angle -= 1) // command to move from 150 degrees to 30 degrees
  {
    servo_x.write(angle);
    delay(15);
  }
  delay(1000);
  
}

#endif // _SERVOCONTROL_H
