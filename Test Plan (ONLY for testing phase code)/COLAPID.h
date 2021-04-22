/*
  COLA's PID
  gRoll  = Inner = x
  gPitch = Outer = y
*/
#ifndef _COLA_PID_h
#define _COLA_PID_h

#include <math.h>

#include "PID_v1.h"
#include "PID_v1_code.h"

double colaPIDr()
{ // Call this function every time you want the roll PID to update. 
  myPIDr.Compute(); // Run the roll PID update code
  return gRoll; // The function returns the roll gymbal angle.
}

double colaPIDp()
{ // Call this function every time you want the pitch PID to update. 
  myPIDp.Compute(); // Run the pitch PID update code
  return gPitch; // The function returns the pitch gymbal angle.
}

void pid_func()
{
  double Pitch = 0; // Current pitch angle in radians
  double Roll = 0; // Current roll angle in radians
  
  double xdot = 0; // Velocity in the x axis (In practice theses are the velocities (either x or y) that the system derives from sensor data)
  double ydot = 0; // Velocity in the y axis
  double target = 0; // Desired x and y velocity value, should always be zero
  double Kp = 1.1; // Proportional gain
  double Ki = 0.1; // Integral gain
  double Kd = 0.5; // Derivative gain
  double gRoll = 0; // Yaw gymbal angle (In practice these gymbal angle values should be fed into Brandon's conversion code to get a servo angle and then the servos should be driven to that angle)
  double gPitch = 0; // Pitch gymbal angle
  double mass = 2.5; // Spaceraft mass in kg
  double thrust = 10; // Spacecraft thrust in N
  double PitchSaturation = 45*pi/180; // Largest possible pitch deflection angle, radians
  double RollSaturation = 45*Pi/180; // Largest possible roll defelection angle, radians

  PID myPIDx(xdot,ax,target,Kp,Ki,Kd,DIRECT); // Initialize x-accel PID
  myPIDx.SetMode(AUTOMATIC); // Start roll PID

  PID myPIDy(ydot,ay,target,Kp,Ki,Kd,DIRECT); // Initialize y-accel PID
  myPIDy.SetMode(AUTOMATIC); // Start pitch PID

  if (ax*mass/thrust >= (pi/2))
  {
    double targetPitch = PitchSaturation;
  }
  else if (ax*mass/thrust <= (-pi/2))
  {
    double targetPitch = -PitchSaturation;
  }
  else
  {
    double targetPitch = asin(ax*mass/thrust);
  }

  if (ay*mass/thrust >= (pi/2))
  {
    double targetRoll = RollSaturation;
  }
  else if (ay*mass/thrust <= (-pi/2))
  {
    double targetRoll = -RollSaturation;
  }
  else
  {
    double targetRoll = asin(ay*mass/thrust);
  }

    PID myPIDr(Pitch,gRoll,targetPitch,Kp,Ki,Kd,DIRECT); // Initialize roll PID
    myPIDr.SetMode(AUTOMATIC); // Start roll PID
    PID myPIDp(Roll,gPitch,targetRoll,Kp,Ki,Kd,DIRECT); // Initialize pitch PID
    myPIDp.SetMode(AUTOMATIC); // Start pitch PID
  
    /* 
        LOOP
    */

  void loop()
  { // This code repeats until the arduino shuts off or explodes or something
    ax = colaPIDx();
    ay = colaPIDy();

    if (ax*mass/thrust >= (pi/2))
    {
      targetPitch = PitchSaturation;
    }
    else if (ax*mass/thrust <= (-pi/2))
    {
      targetPitch = -PitchSaturation;
    }
    else
    {
      targetPitch = asin(ax*mass/thrust);
    }

    if (ay*mass/thrust >= (pi/2))
    {
      targetRoll = RollSaturation;
    }
    else if (ay*mass/thrust <= (-pi/2))
    {
      targetRoll = -RollSaturation;
    }
    else
    {
      targetRoll = asin(ay*mass/thrust);
    }
    
    gRoll = colaPIDr(); // Update the roll gimbal angle
    gPitch = colaPIDp(); // Update the pitch gimbal angle
  }
}

#endif // _COLA_PID_h
