/*
   90 degrees = Center position
   30 degrees = Max angle to the right
   150 degrees = Max angle to the left
   Servo has a total movement of 120 degrees
   gRoll  = Inner = x
   gPitch = Outer = y
*/

#ifndef _SERVOCONTROL_H
#define _SERVOCONTROL_H

#include <Wire.h>
#include <Servo.h>

#include "Gimbal2Servo.h"
#include "COLAPID.h"

Servo servo_x;

int center_position = 90; // 90 degrees is the Center Position
double colaPIDr = 20;     // Gimbal angle at 20 degrees
//double colaPIDp = 20;

void Servo_setup()
{
    servo_x.attach(9);
    servo_x.write(center_position);
}

void Servo_Movement()
{
    //double colaPIDr = colaPIDr();
    //double colaPIDp = colaPIDp();
    for (colaPIDr = 20; colaPIDr >= -20; colaPIDr -= 3)
    {
        double servoangle_InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, colaPIDr); // Inner Gimbal Deflection of x degrees {-20deg to 20deg}
        servo_x.write(servoangle_InnerGimbal);
        delay(500);
    }

    /*for (colaPIDp = 20; colaPIDp <= -20; i += 3)
    {
        double servoangle_OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, colaPIDp); // Outer Gimbal Deflection of x degrees {-20deg to 20deg}
        servo_y.write(servoangle_OuterGimbal);
        delay(500);
    }
    */
    //delay(1000);
}

#endif // _SERVOCONTROL_H
