/*
COLA Arduino
Data Logging code (USB)
*/

#ifndef Data_Logging_h
#define Data_Logging_h

#include <SoftwareSerial.h>
#include "IMU.h"
#include "GPS_I2C.h"


void setup()
{
  SoftwareSerial mySerial(15, 14); // Rx, Tx
  digitalWrite(13, LOW); // switch off LED pin

  Serial.begin(115200); // Initialize UART with baud rate of 115200 bps
}

void loop()
{
  if (Serial.available())
  {
    char data_rcvd = Serial.read();   // Read one byte from serial buffer and save to data_rcvd    
    if (data_rcvd == '1') digitalWrite(13, HIGH); // Switch On
    if (data_rcvd == '0') digitalWrite(13, LOW);  // Switch Off
  }

  if (digitalRead(8) == HIGH) Serial.write('0');    // Send the char '0' to serial if button is not pressed.
  else Serial.write('1');                           // Send the char '1' to serial if button is pressed.
}


#endif