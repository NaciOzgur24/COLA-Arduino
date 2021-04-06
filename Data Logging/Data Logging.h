/*
COLA Arduino
Data Logging code (USB)
*/

#ifndef Data_Logging_h
#define Data_Logging_h

#include <Serial.h>
#include "IMU.h"


void setup()
{
  pinMode(8, INPUT_PULLUP); // set push button pin as input
  pinMode(13, OUTPUT); // set LED pin as output
  digitalWrite(13, LOW); // switch off LED pin

  Serial.begin(115200); // Initialize UART with baud rate of 115200 bps
  Serial1.begin(115200);
}

void loop()
{
  if (Serial1.available()) {
    char data_rcvd = Serial.read();   // Read one byte from serial buffer and save to data_rcvd
    Serial1.print(0)
    
    if (data_rcvd == '1') digitalWrite(13, HIGH); // Switch On
    if (data_rcvd == '0') digitalWrite(13, LOW);  // Switch Off
  }

  if (digitalRead(8) == HIGH) Serial.write('0');    // Send the char '0' to serial if button is not pressed.
  else Serial.write('1');                           // Send the char '1' to serial if button is pressed.
}
