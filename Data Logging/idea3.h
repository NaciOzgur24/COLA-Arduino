/*
COLA Arduino
Data Logging code (USB)
*/

#ifndef Idea_3_h
#define Idea_3_h

#include <SoftwareSerial.h>
#include "IMU.h"
#include "GPS_I2C.h"


void setup()
{
  SoftwareSerial mySerial(15, 14); // Rx, Tx
  digitalWrite(13, LOW); // Low Voltage on Pin 13

  mySerial.begin(115200); // Initialize UART with baud rate of 115200 bps
}

void loop()
{
  if (mySerial.available())
  {
    byte data_rcvd = mySerial.read();   // Read one byte from serial buffer and save to data_rcvd
    mySerial.write(data_rcvd);
    //GPS Section
    long latitude = gps_location(long latitude);





    //IMU Section
    //byte imu_data(byte clear_buffer, byte send_packet, byte Tared_Quaternion, byte status)
    byte Tared_Quaternion = imu_data(byte Tared_Quaternion)

    
  }
}


#endif
