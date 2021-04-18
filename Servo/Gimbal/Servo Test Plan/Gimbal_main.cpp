/*
 * COLA Gimbal2Servo angle main Test Plan code
*/

//#include <Wire.h>
//#include <Servo.h>
//#include <math.h>

#include "Servo_Control_TP_V2.h"

void setup()
{
    Servo_setup();
}

void loop()
{
    // After getting the angle from above it moves the Servo to that angle
    Servo_Control_TP_V2();
}
