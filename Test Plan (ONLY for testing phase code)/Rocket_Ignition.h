/*
  COLA's Rocket Ignition code
  Using: (I2C Protocol)
  Pin 7
*/

#ifndef _ROCKET_IGNITION_h
#define _ROCKET_IGNITION_h

#include <Wire.h>    // Needed for I2C to GNSS

#include "GPS.h" // Gets the Altitude from the GPS sensor ***Still NEED to change from .ino file to either .cpp or .h***


void rocket_ignition_setup()
{
    pinMode(7, OUTPUT);   // Sets the digital pin 7 as output
}

int ignitor()
{
    long altitude_at_ignition = gps_altitude();
    int armed = Ignition_Condition();
    if (armed == 1 && altitude_at_ignition <= 11000) // When COLA is 11 meters off the ground
    {   //NEED 12 Volts
        digitalWrite(7, HIGH); // Sets the digital pin 7 on (Sends high Voltage to the igniter to light it)
        delay(1000);            // Waits 1 seconds after the high voltage is on
        digitalWrite(7, LOW);  // Sets the digital pin 7 off (Turns off the high Voltage)
        return altitude_at_ignition;
    }
}

#endif // _ROCKET_IGNITION_h
