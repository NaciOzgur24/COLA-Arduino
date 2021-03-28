/*
COLA Arduino
Rocket Ignition code at a given Altitude
*/

#include "GPS_I2C.h" // Gets the Altitude from the GPS sensor

int ignite_condition = 0;
int armed = 0;


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
            digitalWrite(13, HIGH); // sets the digital pin 13 on
            delay(2000);            // DOUBLE CHECK
            digitalWrite(13, LOW);  // sets the digital pin 13 off
            ignite_condition = 1;
        }
    }
}
