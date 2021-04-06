/*
COLA Arduino
main code
*/

#include "GPS_I2C.h"
#include "IMU.ino"
#include "Rocket_Ignition.h"



void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}


void loop()
{
  while (/* condition */)
  {
    /* code */
  }

  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(1000);                     // wait for a second
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  delay(1000);                     // wait for a second
}
