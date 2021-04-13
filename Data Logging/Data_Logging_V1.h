/*
COLA Arduino
Data Logging code (USB)
*/

#ifndef Data_Logging_V1_h
#define Data_Logging_V1_h

#include <SoftwareSerial.h>
#include "IMU.h"
#include "GPS_I2C.h"
#include "Rocket_Ignition.h"

// I think portOne is for the IMU sensor
SoftwareSerial portOne(0, 1); // Rx, Tx
// The GPS sensor is a digital sensor on pin 2
//pinMode(2, INPUT);
SoftwareSerial portTwo(15, 14); // Rx, Tx
SoftwareSerial portThree(17, 16); // Rx, Tx

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Waiting for the serial port to connect. Needed for Native USB only
    }
    
    portOne.begin(115200); // Data rate for the SoftwareSerial port
    portTwo.begin(115200);
}

void loop()
{
    // Sensor Data //
    byte Tared_Quaternion = imu_data(byte Tared_Quaternion); // IMU Sensor Quaternion
    long altitude = gps_location(long altitude); // GPS Sensor altitude
    // Sensor Data //


    // For Port 1 //
    portOne.listen();
    Serial.println("Data from port one:");

    // While there's data coming in, it reads it
    while (portOne.available() > 0)
    {
        char inByte = portOne.read();
        Serial.write(inByte);
    }

    Serial.println();
    
    // For Port 2 //
    portTwo.listen();
    Serial.println("Data from port two:");

    while (portTwo.available() > 0)
    {
        char inByte = portTwo.read();
        Serial.write(inByte);
    }

    Serial.println();
}




