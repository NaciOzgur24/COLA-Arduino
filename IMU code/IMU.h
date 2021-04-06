/*
COLA Arduino
IMU Code (Asynchronys Protocol)
*/

#ifndef IMU_h
#define IMU_h

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);


void setup()
{
  //pinMode(0, INPUT_PULLUP); // set push button pin as input
  //pinMode(1, OUTPUT); // set LED pin as output
  SoftwareSerial mySerial(0, 1);
  //digitalWrite(13, LOW); // Low Voltage on Pin 13
  Serial.begin(115200); // Initialize UART with baud rate of 115200 bps
  
  //pinMode(15, INPUT_PULLUP); // set push button pin as input
  //pinMode(14, OUTPUT); // set LED pin as output
  SoftwareSerial mySerial(15, 14);
  //digitalWrite(12, LOW); // Low Voltage on Pin 12
  Serial1.begin(115200); // Initialize UART with baud rate of 115200 bps
}

void loop()
{
  if (Serial.available())
  {
    char data_rcvd = Serial.read(); // Read one byte from serial buffer and save to data_rcvd

    Serial.print();

    if (data_rcvd == '1') digitalWrite(13, HIGH); // High Voltage
    if (data_rcvd == '0') digitalWrite(13, LOW);  // Low Voltage
  }

  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
  Serial.print("Cleared internal buffer. Result: "),Serial.println(result);

  // Send start of packet:
  result = transferByte(0xF6);
  Serial.print("Send start of packet. Result: "),Serial.println(result);

  // Send command (Tared Quaternion)
  result = transferByte(0x00);
  Serial.print("Send commmand 0x00. Result: "),Serial.println(result);

  // Get status of device:
  result = transferByte(0xFF);
  Serial.print("Status of device. Result: "),Serial.println(result);
  
  while (result != 0x01)
  { // Repeat until device is Ready
    result = transferByte(0xFF);
    Serial.print("Status of device. Result: "),Serial.println(result);
  }

  Serial.print("fval 1:"), Serial.println(data[0].fval);
  Serial.print("fval 2:"), Serial.println(data[1].fval);
  Serial.print("fval 3:"), Serial.println(data[2].fval);
  //delay(1000);
}
