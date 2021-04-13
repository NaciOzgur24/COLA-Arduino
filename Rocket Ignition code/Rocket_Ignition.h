/*
COLA Arduino
Rocket Ignition code at a given Altitude
*/

#ifndef Rocket_Ignition_h
#define Rocket_Ignition_h

#include <Wire.h>    // Needed for I2C to GNSS
#include "GPS_I2C.h" // Gets the Altitude from the GPS sensor ***Still NEED to change from .ino file to either .cpp or .h***


void setup()
{
    // ***Prob gonna have to change the pin 13 location because I think it's being used in another location***
    pinMode(13, OUTPUT);   // Sets the digital pin 13 as output
}

void loop()
{
    long altitude = gps_location(long altitude);
    int armed = gps_location(int armed);
    if (armed == 1 && altitude == 11000) // When COLA is 11 meters off the ground
    {
        digitalWrite(13, HIGH); // Sets the digital pin 13 on (Sends high Voltage to the igniter to light it)
        delay(1000);            // Waits 1 seconds after the high voltage is on
        digitalWrite(13, LOW);  // Sets the digital pin 13 off (Turns off the high Voltage)
    }
}


#endif