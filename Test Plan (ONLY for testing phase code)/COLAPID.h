/*
// gRoll  = Inner = x
// gPitch = Outer = y

#ifndef _COLAPID_v1_h
#define _COLAPID_v1_h

#include "PID_v1.h" // This code uses the Arduino PID library. Let's not re-invent the wheel, eh?

double colaPIDr() { // Call this function every time you want the roll PID to update. 
  myPIDr.Compute(); // Run the roll PID update code
      return gRoll // The function returns the roll gymbal angle.
  }

double colaPIDp() { // Call this function every time you want the pitch PID to update. 
  myPIDp.Compute(); // Run the pitch PID update code
      return gPitch // The function returns the pitch gymbal angle.
  }

void main() { // Below code should be integrated into the main program of the Arduino
  double xdot = 0; // Velocity in the x axis (In practice theses are the velocities (either x or y) that the system derives from sensor data)
  double ydot = 0; // Velocity in the y axis
  double target = 0; // Desired velocity value, should always be zero
  double Kp = 1.1; // Proportional gain
  double Ki = 0.1; // Integral gain
  double Kd = 0.5; // Derivative gain
  double gRoll = 0; // Yaw gymbal angle (In practice these gymbal angle values should be fed into Brandon's conversion code to get a servo angle and then the servos should be driven to that angle)
  double gPitch = 0; // Pitch gymbal angle
  
  PID myPIDr(xdot,gRoll,target,Kp,Ki,Kd,DIRECT); // Initialize roll PID
  myPIDr.SetMode(AUTOMATIC); // Start roll PID
  PID myPIDp(ydot,gPitch,target,Kp,Ki,Kd,DIRECT); // Initialize pitch PID
  myPIDp.SetMode(AUTOMATIC); // Start pitch PID
  
  void loop() { // This part of the code repeats until the arduino shuts off or explodes or something
    gRoll = colaPIDr(); // Update the roll gymbal angle
    gPitch = colaPIDp(); // Update the pitch gymbal angle
  }
}

#endif
*/
