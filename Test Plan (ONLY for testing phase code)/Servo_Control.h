/*
   90 degrees = Center position
   30 degrees = Max angle to the right
   150 degrees = Max angle to the left
   Servo has a total movement of 120 degrees
   gRoll  = Inner = servo_x Pin 9
   gPitch = Outer = servo_y Pin 10
*/

#ifndef _SERVOCONTROL_H
#define _SERVOCONTROL_H

#include <Wire.h>
#include <Servo.h>
#include <stdio.h>

#include "Gimbal2Servo.h"
#include "COLAPID.h"

Servo servo_x;
//Servo servo_y;

int center_position_x = 90; // 90 degrees is the Center Position (Upright)
//int center_position_y =; // ***NEED to double check what angle would give the Center Position (Upright)***
double colaPIDr = 20; // ***NEED to change this to get the variable from the PID*** Gimbal Roll angle at 20 degrees (For: Roll/InnerGimbal/Servo_x) 
//double colaPIDp = 20; // ***NEED to change this to get the variable from the PID*** Gimbal Pitch angle at 20 degrees (For: Pitch/OuterGimbal/Servo_y) 

void Servo_setup()
{
  servo_x.attach(9);
  servo_x.write(center_position_x);
  //servo_y.attach(10);
  //servo_y.write(center_position_y);
}

void Servo_Control() // void Servo_Control(gPitch, gRoll)
{
  //double gRoll = colaPIDr(); // This variable comes from the PID controller
  //double gPitch = colaPIDp();
  
  // ***START*** ONLY use this portion of the code for the (Test Plan) phase. It won't work for the real landing!!!
  for (colaPIDr = 20; colaPIDr >= -20; colaPIDr -= 3)
  {
    double servoangle_InnerGimbal_TestPlan = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, colaPIDr); // Inner Gimbal Deflection of 10 degrees {-20deg to 20deg}
    servo_x.write(servoangle_InnerGimbal_TestPlan);
    delay(1000);
  }
  /*
  for (colaPIDp = 20; colaPIDp >= -20; colaPIDp -= 3)
  {
    double servoangle_OuterGimbal_TestPlan = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, colaPIDp); // Outer Gimbal Deflection of 10 degrees {-20deg to 20deg}
    servo_y.write(servoangle_OuterGimbal_TestPlan);
    delay(1000);
  }
  */ // ***END*** of Test Plan code
  

  /* // ***START*** ONLY use this portion of the code for the (Actual Landing). It won't work for the (Test Plan)
  double servoangle_InnerGimbal = gimbal2servo(27.5, 37.9, 11.9, -37.65, -35, gRoll);
  servo_x.write(servoangle_InnerGimbal);
  double servoangle_OuterGimbal = gimbal2servo(25.75, 36.3, 11.9, 36.2, -11, gPitch);
  servo_y.write(servoangle_OuterGimbal);
  delay(15); // IDK how fast the PID controller will spit out an angle ***NEED to ask Bennett***
  */ // ***END*** Actual Landing code
}

#endif // _SERVOCONTROL_H
