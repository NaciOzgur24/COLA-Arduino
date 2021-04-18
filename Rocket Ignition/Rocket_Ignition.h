/*
  COLA Arduino
  Rocket Ignition code at a given Altitude
*/

#ifndef _ROCKET_IGNITION_h
#define _ROCKET_IGNITION_h

#include <Wire.h>    // Needed for I2C to GNSS

#include "GPS.h" // Gets the Altitude from the GPS sensor ***Still NEED to change from .ino file to either .cpp or .h***


void rocket_ignition_setup()
{
    pinMode(7, OUTPUT);   // Sets the digital pin 7 as output
}

long ignitor()
{
    long altitude = gps_altitude();
    int armed = Ignition_Condition();
    if (armed == 1 && altitude <= 11000) // When COLA is 11 meters off the ground
    {   //*** What is the CURRENT being supplied on Pin 7???***
        digitalWrite(7, HIGH); // Sets the digital pin 7 on (Sends high Voltage to the igniter to light it)
        delay(1000);            // Waits 1 seconds after the high voltage is on
        digitalWrite(7, LOW);  // Sets the digital pin 7 off (Turns off the high Voltage)
        long ignited_altitude = altitude; // The altitude the ignitors fired at
        return ignited_altitude;
    }
}

#endif // _ROCKET_IGNITION_h
