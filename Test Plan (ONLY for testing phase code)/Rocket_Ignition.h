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
    pinMode(13, OUTPUT);   // Sets the digital pin 13 as output
}

int ignitor()
{
    long altitude = gps_altitude();
    int armed = Ignition_Condition();
    if (armed == 1 && altitude <= 11000) // When COLA is 11 meters off the ground
    {   //*** What is the CURRENT being supplied on Pin 13???***
        digitalWrite(13, HIGH); // Sets the digital pin 13 on (Sends high Voltage to the igniter to light it)
        delay(1000);            // Waits 1 seconds after the high voltage is on
        digitalWrite(13, LOW);  // Sets the digital pin 13 off (Turns off the high Voltage)
    }
}

#endif
