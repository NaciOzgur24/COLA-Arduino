#include <PID_v1.h>
#include <math.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

double Setpoint, Pitch, gPitch;

double Kp=1.1, Ki=0.1, Kd=0.5;
PID myPIDp(&Pitch, &gPitch, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Pitch = 15; //Arbitrary Start Point
  Setpoint = 0; //Drive towards zero pitch

  myPIDp.SetMode(AUTOMATIC); //start PID
}

void loop()
{
  //get input
  myPIDp.Compute();
  //send output to gymbal servo code
}
