/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <math.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Pitch, gPitch;

//Specify the links and initial tuning parameters
double Kp=1.1, Ki=0.1, Kd=0.5;
PID myPIDp(&Pitch, &gPitch, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Pitch = 15; //Arbitrary Start Point
  Setpoint = 0; //Drive towards zero pitch

  //turn the PID on
  myPIDp.SetMode(AUTOMATIC);
}

void loop()
{
  //Input = get input
  myPIDp.Compute();
  //send output to gymbal servo code
}
