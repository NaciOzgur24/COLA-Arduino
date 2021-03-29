/*
COLA Arduino
Rocket Ignition code at a given Altitude
*/

#include "GPS_I2C.h" // Gets the Altitude from the GPS sensor ***Still NEED to change from .ino file to either .cpp or .h***

int ignite_condition = 0;
// NEED to call variable armed from the GPS_I2C.ino file (i.e. int armed = 0;)

void setup()
{
    pinMode(13, OUTPUT); // sets the digital pin 13 as output
}

void loop()
{
    while (ignite_condition == 0 && armed == 1)
    {
        if (altitude() /*CHANGE*/ == 11000) // When COLA is 11 meters off the ground
        {
            digitalWrite(13, HIGH); // sets the digital pin 13 on (Sends high Voltage to the igniter to light it)
            delay(2000); // waits 2 seconds after the high voltage is on
            digitalWrite(13, LOW);  // sets the digital pin 13 off (Turns off the high Voltage)
            ignite_condition = 1;
        }
    }
}
