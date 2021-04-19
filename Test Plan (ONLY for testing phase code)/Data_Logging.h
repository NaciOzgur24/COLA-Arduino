/*
    COLA's Data Logging code
    Using: (USB so maybe (I2C)??? Protocol)
    Pin (15, 14) (Rx, Tx)
*/

#ifndef _DATALOGGER_h
#define _DATALOGGER_h

#include <Wire.h>
//#include <SoftwareSerial.h> IDK why it doesn't add this library correctly

#include "GPS.h"
#include "IMU.h"
#include "Rocket_Ignition.h"

int landed_condition = 0; // Not Landed = 0. Landed = 1

void Data_Logger_setup()
{
    SoftwareSerial.mySerial(15, 14); // (Rx, Tx)
    mySerial.begin(115200); // Initialize UART with baud rate of 115200 bps
}

void Data_Logger_gps()
{
    if (mySerial.available() > 0) // if (mySerial.available() && altitude <= 0) would stop storing data once its landed
    {
        double latitude = gps_latitude();
        double gps_data_latitude_rcvd = mySerial.read(); // Read double from serial buffer and save to gps_data_latitude_rcvd
        mySerial.write(gps_data_latitude_rcvd);

        double longitude = gps_longitude();
        double gps_data_longitude_rcvd = mySerial.read();
        mySerial.write(gps_data_longitude_rcvd);

        long altitude = gps_altitude();
        long gps_data_altitude_rcvd = mySerial.read();
        mySerial.write(gps_data_altitude_rcvd);
    }
}

/* ???DO WE EVEN NEED TO LOG IMU DATA???
void Data_Logger_imu()
{
    if (mySerial.available() > 0)
    {
        byte quaternion = imu_quaternion();
        byte imu_quaternion_data_rcvd = mySerial.read();
        mySerial.write(imu_quaternion_data_rcvd);
    }
}
*/

void Data_Logger_Rocket_Ignition()
{
    if (mySerial.available() > 0 && landed_condition == 0)
    {
        long altitude_at_ignition = ignitor();
        int rocket_ignition_data_rcvd = mySerial.read(); // Might need to use Wire.read() instead since we have multiple data being stored
        mySerial.write(rocket_ignition_data_rcvd);
        landed_condition += 1;
    }
}

#endif // _DATALOGGER_h
