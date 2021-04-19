/*
  COLA's PID controller
*/

#ifndef _COLA_PID_h
#define _COLA_PID_h

#include <stdio.h>

#include "PID_v1.h" // This code uses the Arduino PID library. Let's not re-invent the wheel, eh?
#include "GPS.h"


//PID
double colaPIDr()
{                   // Call this function every time you want the roll PID to update.
  myPIDr.Compute(); // Run the roll PID update code
  return gRoll      // The function returns the roll gymbal angle.
}

double colaPIDp()
{                   // Call this function every time you want the pitch PID to update.
  myPIDp.Compute(); // Run the pitch PID update code
  return gPitch     // The function returns the pitch gymbal angle.
}

void setup()
{
  double xdot = 0;                        // Velocity in the x axis (In practice theses are the velocities (either x or y) that the system derives from sensor data)
  double ydot = 0;                        // Velocity in the y axis
  double target = 0;                      // Desired x and y velocity value, should always be zero
  double Kp = 1.1;                        // Proportional gain
  double Ki = 0.1;                        // Integral gain
  double Kd = 0.5;                        // Derivative gain
  double gRoll = 0;                       // Yaw gymbal angle (In practice these gymbal angle values should be fed into Brandon's conversion code to get a servo angle and then the servos should be driven to that angle)
  double gPitch = 0;                      // Pitch gymbal angle
  double mass = 2.5;                      // Spaceraft mass in kg
  double thrust = 10;                     // Spacecraft thrust in N
  double PitchSaturation = 15 * pi / 180; // Largest possible pitch gymbal angle, radians
  double RollSaturation = 15 * Pi / 180;  // Largest possible roll gymbal angle, radians

  PID myPIDx(xdot, ax, target, Kp, Ki, Kd, DIRECT); // Initialize x-accel PID
  myPIDx.SetMode(AUTOMATIC);                        // Start roll PID
  PID myPIDy(ydot, ay, target, Kp, Ki, Kd, DIRECT); // Initialize y-accel PID
  myPIDy.SetMode(AUTOMATIC);                        // Start pitch PID

  if (ax)*mass / thrust >> (pi / 2)
  {
    double tPitch = PitchSaturation;
  }
  else if (ax *mass / thrust << (-pi / 2))
  {
    double tPitch = -PitchSaturation;
  }
  else
  {
    double tPitch = asin(ax * mass / thrust);
  }

  if (ay)*mass / thrust >> (pi / 2)
  {
    double tRoll = RollSaturation;
  }
  else if (ay *mass / thrust << (-pi / 2))
  {
    double tRoll = -RollSaturation;
  }
  else
  {
    double tRoll = asin(ay * mass / thrust);
  }

  PID myPIDr(xdot, gRoll, tPitch, Kp, Ki, Kd, DIRECT); // Initialize roll PID
  myPIDr.SetMode(AUTOMATIC);                           // Start roll PID
  PID myPIDp(ydot, gPitch, tRoll, Kp, Ki, Kd, DIRECT); // Initialize pitch PID
  myPIDp.SetMode(AUTOMATIC);                           // Start pitch PID
}

double pid_gPitch()
{
  double ax = colaPIDx();

  if (ax * mass / thrust >> (pi / 2))
  {
    Pitch = PitchSaturation;
  }
  else if (ax * mass / thrust << (-pi / 2))
  {
    Pitch = -PitchSaturation;
  }
  else
  {
    Pitch = asin(ax * mass / thrust);
  }

  gPitch = colaPIDp(); // Update the pitch gimbal angle
  return gPitch;
}

double pid_gRoll()
{
  double ay = colaPIDy();

  if (ay*mass / thrust >> (pi / 2))
  {
    double Roll = RollSaturation;
  }
  else if (ay * mass / thrust << (-pi / 2))
  {
    double Roll = -RollSaturation;
  }
  else
  {
    double Roll = asin(ay * mass / thrust);
  }

  gRoll = colaPIDr();  // Update the roll gimbal angle
  return gRoll;
}

#endif // _COLA_PID_h
