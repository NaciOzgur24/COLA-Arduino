/*
    COLA Arduino
    IMU Code (I2C Protocol)
*/

#ifndef _IMU_H
#define _IMU_H

#include <Wire.h>

#include <BNO055.h>
#include <BNO055_support.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{
    Serial.begin(115200);
    if (!bno.begin())
    {
        Serial.print("No BNO055 detected");
        while (1)
            ;
    }

    delay(1000);


    while (!Serial)
        ; //Wait for user to open terminal
    Serial.println("Starting IMU Testing:");
    Wire.begin();
    //Wire.setClock(400000); //Optional. Increase I2C clock speed to 400kHz.
    
}

void loop()
{
}

#endif // _IMU_H
